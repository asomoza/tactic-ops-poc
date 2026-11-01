// MotionBricks tactics proof of concept: a Wicked Engine window with a soldier and two patrolling enemies on a tiled
// map. A window from SDL2 on Linux and from Win32 on Windows, rendered with Vulkan on both.
//
// Usage: tactics_poc [model_dir] [--capture <dir> [--goal <x> <y>] [--back]]
// model_dir holds the MotionBricks SOMA diffusers weights (root_model/, pose_model/, pose_decoder/).
// --capture saves frames of a scripted move to a tile (8, 5 by default; --back ends with the back to the cover).
//
// A packaged game has assets/ next to the executable, with soma/, shaders/ and models/; it compiles its shaders into
// the user's cache folder. Otherwise the game runs from the source tree.

#ifndef _WIN32
#include <SDL2/SDL.h>
#endif

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>

#include "WickedEngine.h"
#include "game.h"

namespace
{
	class App : public wi::Application
	{
	public:
		explicit App(const tactics::GameSettings& settings) : view(settings) {}
		void Initialize() override
		{
			wi::Application::Initialize();
			wi::initializer::WaitForInitializationsToFinish();
			view.init(canvas);
			view.Load();
			ActivatePath(&view);
			setFrameRateLock(true);
			setTargetFrameRate(60);
		}
		tactics::TacticsView view;
	};

	// Where the game's files are: next to the executable when packaged, in the source tree otherwise. Wicked compiles
	// its shaders from wicked_shaders into shader_cache.
	void ResolvePaths(tactics::GameSettings& settings, std::string& wicked_shaders, std::string& shader_cache)
	{
		namespace fs = std::filesystem;
		const fs::path here = fs::path(wi::helper::GetExecutablePath()).parent_path();
		if (fs::exists(here / "assets" / "character"))
		{
			settings.assets_dir = (here / "assets").string();
			settings.soma_dir = (here / "soma").string();
			settings.motion_shaders = (here / "shaders" / "motion").string();
			if (settings.model_dir.empty())
				settings.model_dir = (here / "models" / "motionbricks_soma_diffusers").string();
			wicked_shaders = (here / "shaders" / "wicked").string() + "/";
			shader_cache = (fs::path(wi::helper::GetCacheDirectoryPath()) / "tactics_poc" / "shaders").string() + "/";
		}
		else
		{
			settings.assets_dir = TACTICS_ROOT "/assets";
			settings.soma_dir = TACTICS_SOMA_DIR;
			if (settings.model_dir.empty())
				settings.model_dir = TACTICS_DEFAULT_MODEL_DIR;
			wicked_shaders = TACTICS_WICKED_SHADER_SOURCE "/";
			shader_cache = TACTICS_BUILD_DIR "/shaders/";
		}
		settings.character = settings.assets_dir + "/character/soldier.glb";
		settings.enemy = settings.assets_dir + "/character/enemy.glb";
	}

	// Scripted captures: wait for the scene to settle, take an idle frame, then move to the goal and take a frame every
	// 0.25 s until the soldier has stopped, and for three seconds after.
	struct Capture
	{
		std::string dir;
		tactics::Tile goal{8, 5};
		bool back = false;
		int ready_frames = 0, shot = 0, stopped_shots = 0;
		float next_shot = 0;
		std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

		// After a frame; returns true when the capture is over.
		bool Step(App& app)
		{
			if (dir.empty())
				return false;
			const float seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
			if (!app.view.error.empty() || seconds > 300)
			{
				std::cerr << "capture: stopping, error '" << app.view.error << "' after " << seconds << " s\n";
				return true;
			}
			if (app.view.Ready() && ++ready_frames == 120)
			{
				const std::string saved = wi::helper::screenshot(app.swapChain, dir + "/shot_00_idle.png");
				std::cerr << "capture: idle frame -> '" << saved << "'\n";
				app.view.MoveTo(goal, back);
				next_shot = seconds + 0.25f;
			}
			else if (ready_frames > 120 && seconds >= next_shot)
			{
				char name[64];
				std::snprintf(name, sizeof(name), "/shot_%02d.png", ++shot);
				const std::string saved = wi::helper::screenshot(app.swapChain, dir + name);
				std::cerr << "capture: frame -> '" << saved << "' moving " << app.view.Moving() << "\n";
				next_shot = seconds + 0.25f;
				if (app.view.Moving())
					stopped_shots = 0;
				else if (shot > 2 && ++stopped_shots > 12)
					return true;
			}
			return false;
		}
	};
}

int main(int argc, char** argv)
{
	tactics::GameSettings settings;
	Capture capture;
	for (int i = 1; i < argc; ++i)
	{
		const std::string arg = argv[i];
		if (arg == "--capture" && i + 1 < argc)
			capture.dir = argv[++i];
		else if (arg == "--goal" && i + 2 < argc)
		{
			capture.goal.x = std::atoi(argv[++i]);
			capture.goal.y = std::atoi(argv[++i]);
		}
		else if (arg == "--back")
			capture.back = true;
		else
			settings.model_dir = arg;
	}
	std::string wicked_shaders, shader_cache;
	ResolvePaths(settings, wicked_shaders, shader_cache);
	std::filesystem::create_directories(shader_cache);
	wi::renderer::SetShaderPath(shader_cache);
	wi::renderer::SetShaderSourcePath(wicked_shaders);

#ifdef _WIN32
	// Vulkan, which the motion module's compute shaders are validated with, instead of Wicked's default DX12.
	wi::arguments::Parse(L"vulkan");
	static App* running = nullptr;
	static bool quit = false;
	App app(settings);
	app.allow_hdr = false;
	running = &app;
	static auto WndProc = [](HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) -> LRESULT {
		switch (message)
		{
		case WM_SIZE:
		case WM_DPICHANGED:
			if (running && running->is_window_active)
				running->SetWindow(hWnd);
			break;
		case WM_INPUT:
			wi::input::rawinput::ParseMessage((void*)lParam);
			break;
		case WM_KEYDOWN:
			if (wParam == VK_ESCAPE)
				quit = true;
			// F11: borderless full screen on the window's monitor, and back.
			else if (wParam == VK_F11 && !(lParam & (1 << 30)))
				wi::platform::SetWindowFullScreen(hWnd, GetWindowLong(hWnd, GWL_STYLE) & WS_OVERLAPPEDWINDOW);
			return DefWindowProc(hWnd, message, wParam, lParam);
		case WM_KILLFOCUS:
			if (running)
				running->is_window_active = false;
			break;
		case WM_SETFOCUS:
			if (running)
				running->is_window_active = true;
			break;
		case WM_DESTROY:
			PostQuitMessage(0);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		return 0;
	};
	SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
	const HINSTANCE instance = GetModuleHandle(nullptr);
	WNDCLASSEXW window_class = {};
	window_class.cbSize = sizeof(window_class);
	window_class.style = CS_HREDRAW | CS_VREDRAW;
	window_class.lpfnWndProc = WndProc;
	window_class.hInstance = instance;
	window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
	window_class.lpszClassName = L"TacticsPoc";
	RegisterClassExW(&window_class);
	RECT rect = {0, 0, 1600, 960};
	AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
	HWND window = CreateWindowW(window_class.lpszClassName, L"MotionBricks tactics PoC", WS_OVERLAPPEDWINDOW,
		CW_USEDEFAULT, 0, rect.right - rect.left, rect.bottom - rect.top, nullptr, nullptr, instance, nullptr);
	ShowWindow(window, SW_SHOWDEFAULT);
	app.SetWindow(window);
	MSG message = {};
	while (message.message != WM_QUIT && !quit)
	{
		if (PeekMessage(&message, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&message);
			DispatchMessage(&message);
		}
		else
		{
			app.Run();
			quit = capture.Step(app) || quit;
		}
	}
	running = nullptr;
	if (!app.view.error.empty())
		std::cerr << "error: " << app.view.error << '\n';
	wi::jobsystem::ShutDown();
#else
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0)
	{
		std::cerr << SDL_GetError() << '\n';
		return 1;
	}
	SDL_Window* window = SDL_CreateWindow("MotionBricks tactics PoC", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1600,
		960, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	if (!window)
	{
		std::cerr << SDL_GetError() << '\n';
		SDL_Quit();
		return 1;
	}
	{
		App app(settings);
		app.allow_hdr = false;
		app.SetWindow(window);
		bool quit = false;
		while (!quit)
		{
			SDL_Event e;
			while (SDL_PollEvent(&e))
			{
				wi::input::sdlinput::ProcessEvent(e);
				if (e.type == SDL_QUIT || (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE))
					quit = true;
				// F11: borderless full screen at the desktop's resolution, and back.
				if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_F11 && !e.key.repeat)
				{
					const bool full = SDL_GetWindowFlags(window) & SDL_WINDOW_FULLSCREEN_DESKTOP;
					SDL_SetWindowFullscreen(window, full ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
				}
				if (e.type == SDL_WINDOWEVENT &&
					(e.window.event == SDL_WINDOWEVENT_RESIZED || e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED))
					app.SetWindow(window);
			}
			if (!quit)
				app.Run();
			quit = capture.Step(app) || quit;
		}
		if (!app.view.error.empty())
			std::cerr << "error: " << app.view.error << '\n';
	}
	SDL_DestroyWindow(window);
	SDL_Quit();
#endif
	return 0;
}
