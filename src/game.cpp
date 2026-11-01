#include "game.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace DirectX;

namespace tactics
{
	namespace
	{
		// A material from a folder of Poly Haven maps: diff.jpg, nor_gl.jpg and arm.jpg (occlusion, roughness, metal).
		wi::ecs::Entity Material(wi::scene::Scene& scene, const std::string& dir)
		{
			const wi::ecs::Entity e = wi::ecs::CreateEntity();
			wi::scene::MaterialComponent& m = scene.materials.Create(e);
			m.textures[wi::scene::MaterialComponent::BASECOLORMAP].name = dir + "/diff.jpg";
			m.textures[wi::scene::MaterialComponent::NORMALMAP].name = dir + "/nor_gl.jpg";
			m.textures[wi::scene::MaterialComponent::SURFACEMAP].name = dir + "/arm.jpg";
			m.SetRoughness(1);
			m.SetMetalness(1);
			m.SetOcclusionEnabled_Primary(true);
			m.CreateRenderData();
			return e;
		}

		// Size a Wicked cube or plane mesh (corners at -1 and 1) to a box in game coordinates and map its texture by
		// world position, one repeat every `tile` meters, so that every face shows it at the same scale.
		void Shape(wi::scene::Scene& scene, wi::ecs::Entity e, const Obstacle& box, wi::ecs::Entity material, float tile)
		{
			wi::scene::MeshComponent& mesh = *scene.meshes.GetComponent(e);
			// Wicked's space mirrors the game's Z.
			const XMFLOAT3 center((box.min.x + box.max.x) * 0.5f, (box.min.y + box.max.y) * 0.5f,
				-(box.min.z + box.max.z) * 0.5f);
			const XMFLOAT3 half((box.max.x - box.min.x) * 0.5f, (box.max.y - box.min.y) * 0.5f,
				(box.max.z - box.min.z) * 0.5f);
			for (size_t i = 0; i < mesh.vertex_positions.size(); ++i)
			{
				XMFLOAT3& p = mesh.vertex_positions[i];
				p = XMFLOAT3(center.x + p.x * half.x, center.y + p.y * half.y, center.z + p.z * half.z);
				const XMFLOAT3& n = mesh.vertex_normals[i];
				XMFLOAT2& uv = mesh.vertex_uvset_0[i];
				if (std::abs(n.y) > 0.5f)
					uv = XMFLOAT2(p.x / tile, p.z / tile);
				else if (std::abs(n.x) > 0.5f)
					uv = XMFLOAT2(p.z / tile, -p.y / tile);
				else
					uv = XMFLOAT2(p.x / tile, -p.y / tile);
			}
			mesh.vertex_tangents.clear();
			scene.materials.Remove(e);
			mesh.subsets[0].materialID = material;
			mesh.CreateRenderData();
		}
		void Line(const Vec3& a, const Vec3& b, const XMFLOAT4& color)
		{
			wi::renderer::RenderableLine line;
			line.start = XMFLOAT3(a.x, a.y, a.z);
			line.end = XMFLOAT3(b.x, b.y, b.z);
			line.color_start = line.color_end = color;
			// Depth tested, so walls hide the lines behind them.
			wi::renderer::DrawLine(line, true);
		}
		// A transparent triangle, depth tested like the lines.
		void Triangle(const Vec3& a, const Vec3& b, const Vec3& c, const XMFLOAT4& color)
		{
			wi::renderer::RenderableTriangle triangle;
			triangle.positionA = XMFLOAT3(a.x, a.y, a.z);
			triangle.positionB = XMFLOAT3(b.x, b.y, b.z);
			triangle.positionC = XMFLOAT3(c.x, c.y, c.z);
			triangle.colorA = triangle.colorB = triangle.colorC = color;
			wi::renderer::DrawTriangle(triangle, false, true);
		}
	}

	TacticsView::TacticsView(const GameSettings& settings) : settings_(settings), grid_(12, 12, 1.5f)
	{
	}

	void TacticsView::BuildMap()
	{
		BuildYard(grid_);

		// Poly Haven textures (CC0), each repeating at its real-world size.
		const std::string textures = settings_.assets_dir + "/textures/";
		const wi::ecs::Entity ground_material = Material(*scene, textures + "concrete_floor_worn_001");
		const wi::ecs::Entity wall_material = Material(*scene, textures + "concrete_block_wall");
		const wi::ecs::Entity low_wall_material = Material(*scene, textures + "brick_wall_02");

		const float extent_x = grid_.Width() * grid_.TileSize(), extent_z = grid_.Height() * grid_.TileSize();
		Shape(*scene, scene->Entity_CreatePlane("ground"), {{-3, 0, -3}, {extent_x + 3, 0, extent_z + 3}}, ground_material,
			3.0f);
		for (int y = 0; y < grid_.Height(); ++y)
			for (int x = 0; x < grid_.Width(); ++x)
			{
				const Tile tile{x, y};
				if (grid_.At(tile) == Cell::Free)
					continue;
				const bool half = grid_.At(tile) == Cell::HalfCover;
				Shape(*scene, scene->Entity_CreateCube(half ? "low wall" : "wall"), grid_.CoverBox(tile),
					half ? low_wall_material : wall_material, 2.0f);
			}
		obstacles_ = grid_.Obstacles();

		const wi::ecs::Entity sun = scene->Entity_CreateLight("sun", XMFLOAT3(0, 20, 0), XMFLOAT3(1, 0.95f, 0.88f), 5, 200,
			wi::scene::LightComponent::DIRECTIONAL);
		wi::scene::TransformComponent* sun_transform = scene->transforms.GetComponent(sun);
		sun_transform->RotateRollPitchYaw(XMFLOAT3(0.9f, 0.6f, 0));
		sun_transform->UpdateTransform();
		scene->lights.GetComponent(sun)->SetCastShadow(true);
		wi::scene::WeatherComponent& weather = scene->weathers.Create(wi::ecs::CreateEntity());
		weather.ambient = XMFLOAT3(0.35f, 0.38f, 0.45f);
		weather.horizon = XMFLOAT3(0.30f, 0.34f, 0.40f);
		weather.zenith = XMFLOAT3(0.12f, 0.17f, 0.26f);
	}

	void TacticsView::Load()
	{
		// MSVC builds leave Wicked's global camera zeroed (fov and clip planes included): start from a fresh one,
		// keeping the render path's size.
		if (camera->fov == 0)
			wi::backlog::post("[tactics] the camera started with every field at 0; reset to the defaults",
				wi::backlog::LogLevel::Warning);
		const float width = camera->width, height = camera->height;
		*camera = wi::scene::CameraComponent();
		camera->width = width;
		camera->height = height;
		camera->SetDirty();
		wi::RenderPath3D::Load();
		setAO(AO_MSAO);
		BuildMap();

		auto fail = [&](const std::string& message) { error = message; };
		if (!LoadSomaSkeleton(settings_.soma_dir + "/soma_skeleton.txt", skeleton_, &error) ||
			!LoadClipSet(settings_.assets_dir + "/motions/kimodo", clips_, &error))
			return;
		enemy_views_.resize(2);
		if (!character_.Load(*scene, settings_.character, skeleton_, &error) ||
			!enemy_views_[0].Load(*scene, settings_.enemy, skeleton_, &error) ||
			!enemy_views_[1].Load(*scene, settings_.enemy, skeleton_, &error))
			return;

		motion::Settings motion_settings;
		motion_settings.model_dir = settings_.model_dir;
		motion_settings.max_batch = 1;
		if (!settings_.motion_shaders.empty())
			motion_settings.shader_dir = settings_.motion_shaders;
		bricks_ = motion::MotionBricks::Create(wi::graphics::GetDevice(), motion_settings, &error);
		if (!bricks_)
			return fail("MotionBricks: " + error);

		planner_ = std::make_unique<MovePlanner>(*bricks_, clips_, grid_, settings_.run_speed);
		// The patrols: random spots anywhere in the yard, starting in the north-west and north-east corners and facing
		// south toward the yard.
		const float south = 3.14159265f;
		const std::vector<Tile> yard = WalkableTiles(grid_, {0, 0}, {grid_.Width() - 1, grid_.Height() - 1});
		patrols_.push_back(std::make_unique<Patrol>(skeleton_, yard, Tile{1, 10}, south));
		patrols_.push_back(std::make_unique<Patrol>(skeleton_, yard, Tile{10, 10}, south));
		seeing_.assign(patrols_.size(), 0);
		Restart();
		ready_ = true;
	}

	bool TacticsView::PointerGround(Vec3& point) const
	{
		const XMFLOAT4 pointer = wi::input::GetPointer();
		const wi::primitive::Ray ray = wi::renderer::GetPickRay(long(pointer.x), long(pointer.y), *this, *camera);
		if (std::abs(ray.direction.y) < 1e-5f)
			return false;
		const float t = -ray.origin.y / ray.direction.y;
		if (t <= 0)
			return false;
		// The view is left-handed with Z mirrored from the game's right-handed coordinates.
		point = {ray.origin.x + ray.direction.x * t, 0, -(ray.origin.z + ray.direction.z * t)};
		return true;
	}

	void TacticsView::Move(Tile goal, bool has_facing, float heading)
	{
		std::vector<Tile> path = grid_.FindPath(unit_tile_, goal);
		if (path.size() < 2)
		{
			status_ = "No path to that tile.";
			return;
		}
		std::vector<Leg> legs = grid_.Legs(path, settings_.max_leg_length);
		// Short moves are walked, through a walking pose halfway along each leg; a direct standing-to-standing
		// transition this short makes the model hop.
		Vec3 previous = grid_.Center(path.front());
		float length = 0;
		for (const Leg& leg : legs)
		{
			length += (leg.end - previous).Length();
			previous = leg.end;
		}
		const bool vaults = std::any_of(legs.begin(), legs.end(), [](const Leg& leg) { return leg.vault; });
		if (crouched_)
			legs = PaceLegs(legs, grid_.Center(path.front()), Pace::Sneak);
		else if (length <= settings_.walk_distance && !vaults)
		{
			std::vector<Leg> walked;
			previous = grid_.Center(path.front());
			for (Leg leg : legs)
			{
				Leg middle;
				middle.end = previous + (leg.end - previous) * 0.5f;
				middle.heading = HeadingOf(leg.end - previous);
				middle.walk = true;
				middle.min_tokens = leg.min_tokens = 8;
				middle.max_tokens = leg.max_tokens = 10;
				middle.path_from = middle.path_to = -1;
				leg.path_from = leg.path_to = -1;
				leg.walk = true;
				previous = leg.end;
				walked.push_back(middle);
				walked.push_back(leg);
			}
			legs = std::move(walked);
		}
		else if (legs.size() == 1 && !vaults)
		{
			// One transition from standing to standing this far leaps; run through the middle instead.
			Leg middle;
			const Vec3 from = grid_.Center(path.front());
			middle.end = from + (legs[0].end - from) * 0.5f;
			middle.heading = HeadingOf(legs[0].end - from);
			legs.insert(legs.begin(), middle);
		}
		const GoalPose pose =
			ChooseGoalPose(grid_, clips_, goal, has_facing ? heading : legs.back().heading, crouched_);
		legs.back().end = pose.ground;
		legs.back().heading = pose.heading;
		std::string plan_error;
		std::vector<Vec3> route;
		for (const Tile& t : path)
			route.push_back(grid_.Center(t));
		if (!planner_->Plan(*player_, legs, route, *pose.clip, obstacles_, reports_, &plan_error))
		{
			status_ = "Transition failed: " + plan_error;
			return;
		}
		path_ = std::move(path);
		follow_ = true;
		legs_ = std::move(legs); // as planned, with any detours
		unit_tile_ = goal;
		unit_heading_ = pose.heading;
		const Cell cover = grid_.CoverAt(goal, nullptr);
		status_ = cover == Cell::HalfCover ? "Moving to half cover." : cover == Cell::FullCover ? "Moving to full cover."
																							   : "Moving.";
	}

	void TacticsView::ToggleCrouch()
	{
		crouched_ = !crouched_;
		if (grid_.CoverAt(unit_tile_, nullptr) != Cell::Free)
		{
			status_ = crouched_ ? "Crouched: moves into the open end crouched." : "Standing: moves into the open end standing.";
			return;
		}
		Leg leg;
		leg.end = grid_.Center(unit_tile_);
		leg.heading = unit_heading_;
		leg.last = true;
		leg.min_tokens = 6;
		leg.max_tokens = 8;
		std::vector<Leg> legs{leg};
		std::string plan_error;
		if (!planner_->Plan(*player_, legs, {leg.end}, crouched_ ? clips_.crouch : clips_.stand, obstacles_, reports_,
				&plan_error))
		{
			crouched_ = !crouched_;
			status_ = "Transition failed: " + plan_error;
			return;
		}
		path_ = {unit_tile_};
		legs_ = std::move(legs);
		status_ = crouched_ ? "Crouching." : "Standing up.";
	}

	void TacticsView::UpdateDetection(float dt)
	{
		// The soldier counts as low when his head is below a standing height: crouched, sneaking or in low cover.
		const Vec3 head = player_->Frame().positions[6];
		const bool low = head.y < 1.3f;
		// Seconds in view to be detected: from 0.6 up close to 1.5 at the edge of the range, twice that when low. With
		// both enemies looking, the meter fills at the rates added.
		float rate = 0;
		const float ease = 1.0f - std::exp(-dt / 0.12f);
		for (size_t i = 0; i < patrols_.size(); ++i)
		{
			view_eyes_[i] = view_eyes_[i] + (patrols_[i]->Eyes() - view_eyes_[i]) * ease;
			view_heading_[i] = WrapAngle(view_heading_[i] + WrapAngle(patrols_[i]->LookHeading() - view_heading_[i]) * ease);
			const Vec3 eyes = view_eyes_[i];
			seeing_[i] = Sees(obstacles_, eyes, view_heading_[i], head, low, settings_.vision);
			if (!seeing_[i])
				continue;
			const float distance = Vec3{head.x - eyes.x, 0, head.z - eyes.z}.Length();
			rate += 1.0f / ((0.6f + 0.9f * distance / settings_.vision.range) * (low ? 2.0f : 1.0f));
		}
		seen_ = rate > 0;
		if (seen_)
			detection_ += dt * rate;
		else
			detection_ = std::max(0.0f, detection_ - 0.5f * dt);
		if (detection_ >= 1)
		{
			detection_ = 1;
			detected_ = true;
			choosing_ = false;
			status_ = "Detected.";
		}
	}

	void TacticsView::Restart()
	{
		// The soldier in his cover pose, facing south with his back to the wall.
		const GoalPose pose = ChooseGoalPose(grid_, clips_, start_tile_, 3.14159265f);
		player_ = std::make_unique<MotionPlayer>(skeleton_);
		player_->SetLoop(pose.clip, pose.clip->loop_start,
			PlaceClipFrame(*pose.clip, pose.clip->loop_start, pose.ground, pose.heading));
		unit_tile_ = start_tile_;
		unit_heading_ = pose.heading;
		crouched_ = false;
		path_.clear();
		legs_.clear();
		reports_.clear();
		for (auto& patrol : patrols_)
			patrol->Reset(grid_, clips_);
		detection_ = 0;
		seen_ = false;
		elapsed_ = 0;
		seeing_.assign(patrols_.size(), 0);
		view_eyes_.clear();
		view_heading_.clear();
		for (auto& patrol : patrols_)
		{
			view_eyes_.push_back(patrol->Eyes());
			view_heading_.push_back(patrol->LookHeading());
		}
		detected_ = false;
		centering_ = true;
		status_ = "Click a free tile to move; hold and drag to choose where to face.";
	}

	void TacticsView::MoveTo(Tile goal, bool back_to_cover)
	{
		float cover_heading = 0;
		const bool has_cover = grid_.CoverAt(goal, &cover_heading) != Cell::Free;
		Move(goal, back_to_cover && has_cover, cover_heading + 3.14159265f);
		const Vec3 center = grid_.Center(goal);
		camera_focus_ = {center.x, 0, center.z};
		camera_distance_ = 6;
	}

	void TacticsView::UpdateCamera(float dt)
	{
		using wi::input::BUTTON;
		const float turn = 1.6f * dt, pan = 9.0f * dt;
		if (wi::input::Press(wi::input::KEYBOARD_BUTTON_SPACE))
			centering_ = true;
		// Any camera input of the player's stops the camera from following the soldier until the next move.
		const BUTTON controls[] = {BUTTON('Q'), BUTTON('E'), BUTTON('R'), BUTTON('F'), BUTTON('W'), BUTTON('A'),
			BUTTON('S'), BUTTON('D'), wi::input::KEYBOARD_BUTTON_UP, wi::input::KEYBOARD_BUTTON_DOWN,
			wi::input::KEYBOARD_BUTTON_LEFT, wi::input::KEYBOARD_BUTTON_RIGHT, wi::input::MOUSE_BUTTON_RIGHT,
			wi::input::MOUSE_BUTTON_MIDDLE};
		if (std::any_of(std::begin(controls), std::end(controls), [](BUTTON b) { return wi::input::Down(b); }) ||
			wi::input::Press(wi::input::MOUSE_SCROLL_AS_BUTTON_UP) || wi::input::Press(wi::input::MOUSE_SCROLL_AS_BUTTON_DOWN))
			follow_ = chasing_ = centering_ = false;
		if (wi::input::Down(BUTTON('Q')))
			camera_yaw_ += turn;
		if (wi::input::Down(BUTTON('E')))
			camera_yaw_ -= turn;
		if (wi::input::Down(BUTTON('R')))
			camera_pitch_ = std::min(1.45f, camera_pitch_ + turn * 0.5f);
		if (wi::input::Down(BUTTON('F')))
			camera_pitch_ = std::max(0.25f, camera_pitch_ - turn * 0.5f);
		if (wi::input::Press(wi::input::MOUSE_SCROLL_AS_BUTTON_UP))
			camera_distance_ = std::max(5.0f, camera_distance_ * 0.9f);
		if (wi::input::Press(wi::input::MOUSE_SCROLL_AS_BUTTON_DOWN))
			camera_distance_ = std::min(40.0f, camera_distance_ * 1.1f);
		// Pan relative to the view on the ground plane (game coordinates, right-handed).
		const float fx = -std::sin(camera_yaw_), fz = -std::cos(camera_yaw_);
		float move_x = 0, move_z = 0;
		if (wi::input::Down(BUTTON('W')) || wi::input::Down(wi::input::KEYBOARD_BUTTON_UP))
			move_x += fx, move_z += fz;
		if (wi::input::Down(BUTTON('S')) || wi::input::Down(wi::input::KEYBOARD_BUTTON_DOWN))
			move_x -= fx, move_z -= fz;
		if (wi::input::Down(BUTTON('A')) || wi::input::Down(wi::input::KEYBOARD_BUTTON_LEFT))
			move_x += fz, move_z -= fx;
		if (wi::input::Down(BUTTON('D')) || wi::input::Down(wi::input::KEYBOARD_BUTTON_RIGHT))
			move_x -= fz, move_z += fx;
		camera_focus_.x += move_x * pan;
		camera_focus_.z += move_z * pan;

		// Mouse: right drag orbits and tilts, middle drag pans so the ground follows the cursor.
		const XMFLOAT4 pointer = wi::input::GetPointer();
		const float dx = pointer.x - last_pointer_.x, dy = pointer.y - last_pointer_.y;
		last_pointer_ = XMFLOAT2(pointer.x, pointer.y);
		if (wi::input::Down(wi::input::MOUSE_BUTTON_RIGHT))
		{
			camera_yaw_ -= dx * 0.006f;
			camera_pitch_ = std::clamp(camera_pitch_ + dy * 0.004f, 0.25f, 1.45f);
		}
		if (wi::input::Down(wi::input::MOUSE_BUTTON_MIDDLE))
		{
			const float meters_per_pixel = camera_distance_ * 0.0012f;
			// Right on the ground is (-fz, fx); forward is (fx, fz).
			camera_focus_.x += (fz * dx + fx * dy) * meters_per_pixel;
			camera_focus_.z += (-fx * dx + fz * dy) * meters_per_pixel;
		}

		// Camera position in game coordinates, then into the mirrored view space.
		const float h = std::cos(camera_pitch_) * camera_distance_;
		const XMVECTOR focus = XMVectorSet(camera_focus_.x, 0.8f, -camera_focus_.z, 0);
		const XMVECTOR eye = XMVectorSet(camera_focus_.x + std::sin(camera_yaw_) * h,
			0.8f + std::sin(camera_pitch_) * camera_distance_, -(camera_focus_.z + std::cos(camera_yaw_) * h), 0);
		camera->TransformCamera(XMMatrixInverse(nullptr, XMMatrixLookAtLH(eye, focus, XMVectorSet(0, 1, 0, 0))));
		camera->UpdateCamera();
	}

	void TacticsView::FollowSoldier(float dt)
	{
		// Where the soldier is on screen, from -1 to 1 across; the view mirrors Z.
		const Vec3 hips = player_->Frame().positions[0];
		const XMVECTOR screen =
			XMVector3TransformCoord(XMVectorSet(hips.x, hips.y, -hips.z, 1), camera->GetViewProjection());
		const float off_center = std::max(std::abs(XMVectorGetX(screen)), std::abs(XMVectorGetY(screen)));
		if (centering_)
			centering_ = off_center > 0.05f;
		// Start sliding when he leaves the middle 60% of the screen, then keep him in view for the rest of the move and
		// settle once he has stopped.
		if (!follow_)
			chasing_ = false;
		else if (player_->Busy())
			chasing_ = chasing_ || off_center > 0.6f;
		else
			chasing_ = chasing_ && off_center > 0.1f;
		if (!centering_ && !chasing_)
			return;
		const float rate = 1.0f - std::exp(-dt / 0.15f);
		camera_focus_.x += (hips.x - camera_focus_.x) * rate;
		camera_focus_.z += (hips.z - camera_focus_.z) * rate;
	}

	void TacticsView::UpdateInput()
	{
		Vec3 ground;
		const bool on_ground = PointerGround(ground);
		Tile hovered;
		const bool on_tile = on_ground && grid_.TileAt(ground, hovered);
		const bool left = wi::input::Down(wi::input::MOUSE_BUTTON_LEFT);
		if (!choosing_ && !player_->Busy() && wi::input::Press(wi::input::BUTTON('C')) && !GetGUI().HasFocus())
			ToggleCrouch();
		if (!choosing_)
		{
			if (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) && on_tile && !player_->Busy() && hovered != unit_tile_ &&
				grid_.Walkable(hovered) && !grid_.FindPath(unit_tile_, hovered).empty() && !GetGUI().HasFocus())
			{
				choosing_ = true;
				choice_tile_ = hovered;
				choice_has_facing_ = false;
			}
			return;
		}
		if (left)
		{
			// Beyond a small dead zone around the tile center, the cursor sets the facing: 8 directions, free with
			// Shift held.
			const Vec3 center = grid_.Center(choice_tile_);
			const Vec3 offset = Vec3{ground.x, 0, ground.z} - center;
			choice_has_facing_ = on_ground && offset.Length() > grid_.TileSize() * 0.3f;
			if (choice_has_facing_)
			{
				float heading = HeadingOf(offset);
				const bool free = wi::input::Down(wi::input::KEYBOARD_BUTTON_LSHIFT) ||
					wi::input::Down(wi::input::KEYBOARD_BUTTON_RSHIFT);
				if (!free)
				{
					const float step = 3.14159265f / 4.0f;
					heading = std::round(heading / step) * step;
				}
				choice_heading_ = heading;
			}
			return;
		}
		choosing_ = false;
		Move(choice_tile_, choice_has_facing_, choice_heading_);
	}

	void TacticsView::DrawOverlay(const Tile* hovered) const
	{
		// Lines are drawn in view space, which mirrors Z.
		auto view = [](Vec3 p) { return Vec3{p.x, p.y, -p.z}; };
		const float size = grid_.TileSize();
		const XMFLOAT4 grid_color(1, 1, 1, 0.18f);
		for (int i = 0; i <= grid_.Width(); ++i)
			Line(view({i * size, 0.02f, 0}), view({i * size, 0.02f, grid_.Height() * size}), grid_color);
		for (int i = 0; i <= grid_.Height(); ++i)
			Line(view({0, 0.02f, i * size}), view({grid_.Width() * size, 0.02f, i * size}), grid_color);

		auto outline = [&](Tile t, const XMFLOAT4& color) {
			const float x0 = t.x * size + 0.06f, x1 = (t.x + 1) * size - 0.06f;
			const float z0 = t.y * size + 0.06f, z1 = (t.y + 1) * size - 0.06f;
			Line(view({x0, 0.03f, z0}), view({x1, 0.03f, z0}), color);
			Line(view({x1, 0.03f, z0}), view({x1, 0.03f, z1}), color);
			Line(view({x1, 0.03f, z1}), view({x0, 0.03f, z1}), color);
			Line(view({x0, 0.03f, z1}), view({x0, 0.03f, z0}), color);
		};
		if (hovered && grid_.Walkable(*hovered))
		{
			const Cell cover = grid_.CoverAt(*hovered, nullptr);
			const XMFLOAT4 color = cover == Cell::FullCover ? XMFLOAT4(0.3f, 0.6f, 1, 1)
				: cover == Cell::HalfCover                   ? XMFLOAT4(0.4f, 0.9f, 1, 1)
															 : XMFLOAT4(1, 1, 1, 0.8f);
			outline(*hovered, color);
			if (!player_->Busy())
			{
				const std::vector<Tile> preview = grid_.FindPath(unit_tile_, *hovered);
				for (size_t i = 1; i < preview.size(); ++i)
					Line(view(grid_.Center(preview[i - 1]) + Vec3{0, 0.05f, 0}),
						view(grid_.Center(preview[i]) + Vec3{0, 0.05f, 0}), XMFLOAT4(0.4f, 0.9f, 1, 0.7f));
			}
		}
		// The goal being chosen, with the facing arrow while dragging.
		if (choosing_)
		{
			outline(choice_tile_, XMFLOAT4(1, 0.85f, 0.3f, 1));
			if (choice_has_facing_)
			{
				const Vec3 c = grid_.Center(choice_tile_) + Vec3{0, 0.08f, 0};
				const Vec3 f{std::sin(choice_heading_), 0, std::cos(choice_heading_)};
				const Vec3 side{f.z, 0, -f.x};
				const Vec3 tip = c + f * 0.9f;
				const XMFLOAT4 color(1, 0.85f, 0.3f, 1);
				Line(view(c), view(tip), color);
				Line(view(tip), view(tip - f * 0.25f + side * 0.15f), color);
				Line(view(tip), view(tip - f * 0.25f - side * 0.15f), color);
			}
		}
		// The legs of the current move: waypoints where one transition ends and the next begins.
		if (debug_ && player_->Busy())
		{
			Vec3 previous = grid_.Center(path_.front());
			for (const Leg& leg : legs_)
			{
				Line(view(previous + Vec3{0, 0.06f, 0}), view(leg.end + Vec3{0, 0.06f, 0}),
					leg.detour ? XMFLOAT4(1, 0.35f, 0.9f, 1) : XMFLOAT4(1, 0.8f, 0.2f, 1));
				const Vec3 facing{std::sin(leg.heading), 0, std::cos(leg.heading)};
				Line(view(leg.end + Vec3{0, 0.06f, 0}), view(leg.end + facing * 0.6f + Vec3{0, 0.06f, 0}),
					XMFLOAT4(1, 0.4f, 0.2f, 1));
				previous = leg.end;
			}
		}
		// Each enemy's view, shaded on the ground and cut off where cover blocks it: stronger where he sees everyone
		// (up to the first wall or low wall), fainter beyond low walls where he still sees a standing soldier (up to the
		// first wall). Yellow, or red while he sees the soldier.
		for (size_t i = 0; i < patrols_.size(); ++i)
		{
			const Vec3 center{view_eyes_[i].x, 0.05f, view_eyes_[i].z};
			const float heading = view_heading_[i], half = settings_.vision.half_angle, range = settings_.vision.range;
			const XMFLOAT4 strong = seeing_[i] ? XMFLOAT4(1, 0.15f, 0.1f, 0.3f) : XMFLOAT4(1, 0.85f, 0.25f, 0.22f);
			const XMFLOAT4 faint = seeing_[i] ? XMFLOAT4(1, 0.15f, 0.1f, 0.12f) : XMFLOAT4(1, 0.85f, 0.25f, 0.08f);
			auto at = [&](float angle, float distance) {
				return center + Vec3{std::sin(angle), 0, std::cos(angle)} * distance;
			};
			// Rays at fixed world angles every 1.5 degrees, plus the two edges, so that the cut-offs at walls stay put
			// while the view turns.
			constexpr float kStep = 3.14159265f / 120.0f;
			std::vector<float> angles{heading - half};
			for (float a = std::ceil((heading - half) / kStep) * kStep; a < heading + half; a += kStep)
				if (a > angles.back() + 1e-4f)
					angles.push_back(a);
			angles.push_back(heading + half);
			Vec3 previous_low, previous_standing;
			for (size_t k = 0; k < angles.size(); ++k)
			{
				const float angle = angles[k];
				const Vec3 low = at(angle, SightDistance(obstacles_, center, angle, range, true));
				const Vec3 standing = at(angle, SightDistance(obstacles_, center, angle, range, false));
				if (k > 0)
				{
					Triangle(view(center), view(previous_low), view(low), strong);
					Triangle(view(previous_low), view(previous_standing), view(standing), faint);
					Triangle(view(previous_low), view(standing), view(low), faint);
				}
				previous_low = low;
				previous_standing = standing;
			}
		}
	}

	void TacticsView::Update(float dt)
	{
		UpdateCamera(dt);
		if (ready_)
		{
			Vec3 ground;
			Tile hovered;
			const bool has_hover = PointerGround(ground) && grid_.TileAt(ground, hovered);
			has_hover_ = has_hover;
			hover_tile_ = hovered;
			if (wi::input::Press(wi::input::KEYBOARD_BUTTON_F1))
				debug_ = !debug_;
			if (detected_)
			{
				// Paused on the detection until the player clicks.
				if (wi::input::Press(wi::input::MOUSE_BUTTON_LEFT) && !GetGUI().HasFocus())
					Restart();
			}
			else
			{
				elapsed_ += dt;
				UpdateInput();
				player_->Advance(dt);
				for (auto& patrol : patrols_)
				{
					std::string patrol_error;
					if (!patrol->Update(dt, *planner_, grid_, clips_, obstacles_, &patrol_error))
						status_ = "Patrol: " + patrol_error;
				}
				UpdateDetection(dt);
			}
			FollowSoldier(dt);
			PoseWorld(skeleton_, player_->Pose(), world_);
			// The game is right-handed; the view mirrors Z.
			character_.Apply(*scene, world_);
			for (size_t i = 0; i < patrols_.size(); ++i)
			{
				PoseWorld(skeleton_, patrols_[i]->Player().Pose(), enemy_world_);
				enemy_views_[i].Apply(*scene, enemy_world_);
			}
			DrawOverlay(has_hover ? &hovered : nullptr);
			if (!detected_ && !player_->Busy() && status_.rfind("Moving", 0) == 0)
				status_ = "Click a free tile to move; hold and drag to choose where to face.";
		}
		wi::RenderPath3D::Update(dt);
	}

	void TacticsView::Compose(wi::graphics::CommandList cmd) const
	{
		wi::RenderPath3D::Compose(cmd);
		wi::font::Params p(20, 16, 22);
		p.color = wi::Color(235, 240, 250);
		p.shadowColor = wi::Color(0, 0, 0, 220);
		if (!error.empty())
		{
			wi::font::Draw("Error: " + error, p, cmd);
			return;
		}
		const float width = GetLogicalWidth(), height = GetLogicalHeight();
		char text[64];
		auto format_time = [&](float seconds) {
			std::snprintf(text, sizeof(text), "%d:%06.3f", int(seconds) / 60, std::fmod(seconds, 60.0f));
			return std::string(text);
		};

		// Always on: the round's time at the top center, and the detection meter under it.
		wi::font::Params big(width * 0.5f, 18, 40);
		big.h_align = wi::font::WIFALIGN_CENTER;
		big.color = wi::Color(240, 240, 245);
		big.shadowColor = wi::Color(0, 0, 0, 220);
		if (!detected_)
			wi::font::Draw(format_time(elapsed_), big, cmd);
		if (detection_ > 0 && !detected_)
		{
			wi::font::Params meter(width * 0.5f, 66, 18);
			meter.h_align = wi::font::WIFALIGN_CENTER;
			meter.color = seen_ ? wi::Color(255, 90, 70) : wi::Color(255, 220, 120);
			meter.shadowColor = wi::Color(0, 0, 0, 220);
			const int filled = int(detection_ * 20 + 0.5f);
			wi::font::Draw("[" + std::string(filled, '#') + std::string(20 - filled, '.') + "]" +
					(seen_ ? "  SEEN" : ""),
				meter, cmd);
		}

		if (!debug_)
		{
			p.size = 15;
			p.color = wi::Color(235, 240, 250, 160);
			wi::font::Draw("F1: help", p, cmd);
		}
		else
		{
			wi::font::Draw("MotionBricks tactics proof of concept", p, cmd);
			p.posY += 30;
			p.size = 15;
			wi::font::Draw("Left click: move   Right drag: rotate and tilt   Middle drag: pan   Wheel: zoom   Space: center   "
						   "C: crouch   F1: help   F11: full screen   (keys: WASD/arrows pan, Q/E rotate, R/F tilt)",
				p, cmd);
			p.posY += 20;
			wi::font::Draw("Cyan outline: half cover, blue: full cover", p, cmd);
			p.posY += 24;
			wi::font::Draw(status_ + (has_hover_ ? "   [tile " + std::to_string(hover_tile_.x) + ", " +
											  std::to_string(hover_tile_.y) + "]"
											: ""),
				p, cmd);
			if (!reports_.empty())
			{
				p.posY += 30;
				wi::font::Draw("Last move: one transition per leg (pink legs end at a path tile added to keep clear of cover)",
					p, cmd);
				for (size_t i = 0; i < reports_.size(); ++i)
				{
					const LegReport& r = reports_[i];
					char line[160];
					std::snprintf(line, sizeof(line), "  leg %zu%s: %.1f m in %d frames (%d tokens), generated in %.1f ms%s",
						i + 1, r.detour ? " (detour)" : "", r.distance, r.frames, r.tokens, r.milliseconds,
						r.contact_frames ? (", touches cover in " + std::to_string(r.contact_frames) + " frames, " +
											   std::to_string(int(r.depth * 100)) + " cm deep")
											   .c_str()
										 : "");
					p.posY += 20;
					wi::font::Draw(line, p, cmd);
				}
			}
		}

		// Game over: the view darkened, the time survived, and how to play again.
		if (detected_)
		{
			wi::image::Params shade(0, 0, width, height);
			shade.color = XMFLOAT4(0.05f, 0.0f, 0.0f, 0.65f);
			shade.blendFlag = wi::enums::BLENDMODE_ALPHA;
			wi::image::Draw(wi::texturehelper::getWhite(), shade, cmd);
			wi::font::Params title(width * 0.5f, height * 0.38f, 96);
			title.h_align = wi::font::WIFALIGN_CENTER;
			title.v_align = wi::font::WIFALIGN_CENTER;
			title.color = wi::Color(235, 60, 50);
			title.shadowColor = wi::Color(0, 0, 0, 255);
			wi::font::Draw("GAME OVER", title, cmd);
			wi::font::Params line(width * 0.5f, height * 0.38f + 90, 32);
			line.h_align = wi::font::WIFALIGN_CENTER;
			line.color = wi::Color(240, 240, 245);
			line.shadowColor = wi::Color(0, 0, 0, 255);
			wi::font::Draw("You were found after " + format_time(elapsed_), line, cmd);
			line.posY += 56;
			line.size = 22;
			line.color = wi::Color(200, 200, 210);
			wi::font::Draw("Click to play again", line, cmd);
		}
	}
}
