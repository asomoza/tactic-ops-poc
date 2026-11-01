// Measures short moves: from standing to standing 1 to 3 tiles away, straight ahead or sideways, with different
// allowed transition lengths. Reports airborne frames (both feet off the ground), steps and peak speed.
//
// Usage: short_moves [model_dir]

#include <algorithm>
#include <cmath>
#include <iterator>
#include <cstdio>
#include <memory>

#include "grid.h"
#include "motion_player.h"
#include "rig.h"
#include "wiGraphicsDevice_Vulkan.h"
#include "wiJobSystem.h"

using namespace tactics;

int main(int argc, char** argv)
{
	const std::string model_dir = argc > 1 ? argv[1] : TACTICS_DEFAULT_MODEL_DIR;
	std::string error;
	SomaSkeleton skeleton;
	ClipSet clips;
	if (!LoadSomaSkeleton(TACTICS_SOMA_DIR "/soma_skeleton.txt", skeleton, &error) ||
		!LoadClipSet(TACTICS_ROOT "/assets/motions/kimodo", clips, &error))
	{
		std::printf("load failed: %s\n", error.c_str());
		return 1;
	}
	wi::jobsystem::Initialize();
	auto device = std::make_unique<wi::graphics::GraphicsDevice_Vulkan>(nullptr);
	wi::graphics::GetDevice() = device.get();
	motion::Settings settings;
	settings.model_dir = model_dir;
	settings.max_batch = 1;
	auto bricks = motion::MotionBricks::Create(device.get(), settings, &error);
	if (!bricks)
	{
		std::printf("MotionBricks: %s\n", error.c_str());
		return 1;
	}
	Grid grid(12, 12, 1.5f);
	const Tile start{5, 5};
	// Variants: direct (no middle pose), or a middle pose walking at a given speed or jogging.
	struct Variant
	{
		const char* name;
		int middle; // 0 none, 1 walk, 2 run
		float speed;
	} variants[] = {{"walk 0.9", 1, 0.9f}, {"walk 1.2", 1, 1.2f}};
	const std::pair<int, int> ranges[] = {{8, 14}, {6, 10}, {8, 10}, {6, 8}};
	struct Case
	{
		const char* name;
		int dx, dy;
	} cases[] = {{"1 ahead", 0, 1}, {"2 ahead", 0, 2}, {"3 ahead", 0, 3}, {"1 side", 1, 0}, {"2 side", 2, 0},
		{"3 side", 3, 0}, {"1 diag", 1, 1}};
	const char* wanted[] = {"1 ahead", "2 ahead", "1 side", "2 side", "1 diag"};
	for (const Case& c : cases)
	{
		if (std::none_of(std::begin(wanted), std::end(wanted), [&](const char* w) { return std::string(w) == c.name; }))
			continue;
		for (const Variant& variant : variants)
		for (const auto& [low, high] : ranges)
		{
			MotionPlayer player(skeleton);
			// Standing on the start tile facing +Z (heading 0).
			player.SetLoop(&clips.stand, 0, PlaceClipFrame(clips.stand, 0, grid.Center(start), 0));
			player.Advance(0.2f);
			MovePlanner planner(*bricks, clips, grid, variant.middle == 2 ? variant.speed : 3.0f);
			planner.min_tokens = low;
			planner.max_tokens = high;
			planner.walk_speed = variant.speed;
			const Tile goal{start.x + c.dx, start.y + c.dy};
			std::vector<Leg> legs = grid.Legs(grid.FindPath(start, goal), 5.0f);
			if (variant.middle)
			{
				const Vec3 from = grid.Center(start), to = grid.Center(goal);
				Leg middle;
				middle.end = from + (to - from) * 0.5f;
				middle.heading = HeadingOf(to - from);
				middle.walk = variant.middle == 1;
				legs.insert(legs.begin(), middle);
			}
			std::vector<LegReport> reports;
			if (!planner.Plan(player, legs, {}, clips.stand, {}, reports, &error))
			{
				std::printf("plan failed: %s\n", error.c_str());
				return 1;
			}
			int frames = 0, airborne = 0, steps = 0;
			float peak = 0, bounce = 0;
			Vec3 previous = player.Frame().positions[0];
			const float rest_height = previous.y;
			bool left_down = true, right_down = true;
			while (player.Busy())
			{
				player.Advance(1.0f / 30.0f);
				const ModelFrame& f = player.Frame();
				const float left = std::min(f.positions[24].y, f.positions[25].y);
				const float right = std::min(f.positions[28].y, f.positions[29].y);
				const bool l = left < 0.06f, r = right < 0.06f;
				airborne += !l && !r;
				steps += (l && !left_down) + (r && !right_down);
				left_down = l;
				right_down = r;
				const Vec3 hips = f.positions[0];
				peak = std::max(peak, std::hypot(hips.x - previous.x, hips.z - previous.z) * 30);
				bounce = std::max(bounce, hips.y - rest_height);
				previous = hips;
				++frames;
			}
			std::printf("%-8s %-10s tokens %2d-%-2d: legs %zu, %.2f s; airborne %2d frames, %d steps, peak %.2f m/s\n",
				c.name, variant.name, low, high, reports.size(), frames / 30.0f, airborne, steps, peak);
		}
	}
	return 0;
}
