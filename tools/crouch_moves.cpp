// Measures crouching and standing up in place with MotionBricks (from five moments of the idle, with different
// allowed transition lengths: the length, how far each foot slides or steps, how the hips move), sneaking moves, and
// vaults over a low wall from standing, after sneaking and from a run.
//
// Usage: crouch_moves [model_dir]

#include <algorithm>
#include <cmath>
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
	const Vec3 ground = grid.Center({5, 5});
	const std::pair<int, int> ranges[] = {{6, 6}, {6, 8}, {6, 10}, {8, 14}};
	const float waits[] = {0.2f, 1.0f, 2.0f, 3.0f, 4.5f}; // seconds of idle before the transition
	// SOMA30 joints: left heel and toe 24, 25; right heel and toe 28, 29.
	const int feet[2][2] = {{24, 25}, {28, 29}};
	for (int crouching = 1; crouching >= 0; --crouching)
	{
		const MotionClip& from = crouching ? clips.stand : clips.crouch;
		const MotionClip& to = crouching ? clips.crouch : clips.stand;
		std::printf("%s\n", crouching ? "stand -> crouch" : "crouch -> stand");
		for (const auto& [low, high] : ranges)
		{
			for (float wait : waits)
			{
				MotionPlayer player(skeleton);
				player.SetLoop(&from, from.loop_start, PlaceClipFrame(from, from.loop_start, ground, 0));
				player.Advance(wait);
				MovePlanner planner(*bricks, clips, grid, 3.0f);
				Leg leg;
				leg.end = ground;
				leg.last = true;
				leg.min_tokens = low;
				leg.max_tokens = high;
				std::vector<Leg> legs{leg};
				std::vector<LegReport> reports;
				if (!planner.Plan(player, legs, {ground}, to, {}, reports, &error))
				{
					std::printf("plan failed: %s\n", error.c_str());
					return 1;
				}
				// Feet: the farthest each moves on the ground from where it started, and how high it rises.
				float start[2][2], slide[2] = {0, 0};
				float peak[2] = {0, 0}, rest[2];
				const ModelFrame& first = player.Frame();
				for (int f = 0; f < 2; ++f)
				{
					start[f][0] = first.positions[feet[f][0]].x;
					start[f][1] = first.positions[feet[f][0]].z;
					rest[f] = std::min(first.positions[feet[f][0]].y, first.positions[feet[f][1]].y);
				}
				const float hips_start = first.positions[0].y;
				float hips_min = hips_start, hips_max = hips_start, speed = 0, previous = hips_start;
				int frames = 0;
				while (player.Busy())
				{
					player.Advance(1.0f / 30.0f);
					const ModelFrame& frame = player.Frame();
					for (int f = 0; f < 2; ++f)
					{
						const Vec3& heel = frame.positions[feet[f][0]];
						slide[f] = std::max(slide[f], std::hypot(heel.x - start[f][0], heel.z - start[f][1]));
						const float height = std::min(heel.y, frame.positions[feet[f][1]].y);
						peak[f] = std::max(peak[f], height - rest[f]);
					}
					const float hips = frame.positions[0].y;
					hips_min = std::min(hips_min, hips);
					hips_max = std::max(hips_max, hips);
					speed = std::max(speed, std::abs(hips - previous) * 30);
					previous = hips;
					++frames;
				}
				const float hips_end = player.Frame().positions[0].y;
				// The hips going past their end height before settling, in centimeters.
				const float overshoot = crouching ? (hips_end - hips_min) * 100 : (hips_max - hips_end) * 100;
				std::printf("  tokens %2d-%-2d idle %.1f s: %2d tokens, %.2f s; feet move %4.1f / %4.1f cm, rise %3.1f / %3.1f cm; "
							"hips %.2f -> %.2f m, peak %.2f m/s, overshoot %.1f cm\n",
					low, high, wait, reports[0].tokens, frames / 30.0f, slide[0] * 100, slide[1] * 100, peak[0] * 100, peak[1] * 100,
					hips_start, hips_end, speed, overshoot);
			}
		}
	}

	// Crouched moves: sneaking all the way, from the crouch idle to the crouch idle.
	std::printf("sneaking\n");
	const Tile start{2, 2};
	const Tile goals[] = {{2, 4}, {2, 5}, {2, 7}, {5, 5}, {6, 2}};
	for (const Tile& goal : goals)
	{
		MotionPlayer player(skeleton);
		player.SetLoop(&clips.crouch, clips.crouch.loop_start,
			PlaceClipFrame(clips.crouch, clips.crouch.loop_start, grid.Center(start), 0));
		player.Advance(0.5f);
		MovePlanner planner(*bricks, clips, grid, 3.0f);
		const std::vector<Tile> path = grid.FindPath(start, goal);
		std::vector<Leg> legs = PaceLegs(grid.Legs(path, 5.0f), grid.Center(start), Pace::Sneak);
		const GoalPose pose = ChooseGoalPose(grid, clips, goal, legs.back().heading, true);
		legs.back().end = pose.ground;
		legs.back().heading = pose.heading;
		std::vector<Vec3> route;
		for (const Tile& t : path)
			route.push_back(grid.Center(t));
		std::vector<LegReport> reports;
		if (!planner.Plan(player, legs, route, *pose.clip, {}, reports, &error))
		{
			std::printf("plan failed: %s\n", error.c_str());
			return 1;
		}
		int frames = 0, airborne = 0, steps = 0;
		float hips_low = 10, hips_high = 0, peak = 0, distance = 0;
		bool left_down = true, right_down = true;
		Vec3 previous = player.Frame().positions[0];
		while (player.Busy())
		{
			player.Advance(1.0f / 30.0f);
			const ModelFrame& f = player.Frame();
			const bool l = std::min(f.positions[24].y, f.positions[25].y) < 0.06f;
			const bool r = std::min(f.positions[28].y, f.positions[29].y) < 0.06f;
			airborne += !l && !r;
			steps += (l && !left_down) + (r && !right_down);
			left_down = l;
			right_down = r;
			const Vec3 hips = f.positions[0];
			hips_low = std::min(hips_low, hips.y);
			hips_high = std::max(hips_high, hips.y);
			const float step = std::hypot(hips.x - previous.x, hips.z - previous.z);
			distance += step;
			peak = std::max(peak, step * 30);
			previous = hips;
			++frames;
		}
		std::printf("  to (%d, %d), %zu path tiles: %zu legs, %.1f s, %.1f m at %.2f m/s (peak %.2f); hips %.2f..%.2f m; "
					"%d steps, airborne %d frames\n",
			goal.x, goal.y, path.size(), reports.size(), frames / 30.0f, distance, distance / (frames / 30.0f), peak,
			hips_low, hips_high, steps, airborne);
	}

	// Vaults over a low wall across the yard, from (2, 6) to (10, 6), so that going around costs more than vaulting.
	std::printf("vaults\n");
	Grid yard(12, 12, 1.5f);
	for (int x = 2; x <= 10; ++x)
		yard.Set({x, 6}, Cell::HalfCover);
	const std::vector<Obstacle> walls = yard.Obstacles();
	struct VaultCase
	{
		const char* name;
		Tile from, to;
		bool crouched;
	} vault_cases[] = {{"standing at the wall", {7, 5}, {7, 7}, false}, {"crouched at the wall", {7, 5}, {7, 8}, true},
		{"sneaking up", {7, 2}, {7, 8}, true}, {"running", {7, 1}, {7, 9}, false}};
	for (const VaultCase& c : vault_cases)
	{
		const MotionClip& idle = c.crouched ? clips.crouch : clips.stand;
		MotionPlayer player(skeleton);
		player.SetLoop(&idle, idle.loop_start, PlaceClipFrame(idle, idle.loop_start, yard.Center(c.from), 0));
		player.Advance(0.5f);
		MovePlanner planner(*bricks, clips, yard, 3.0f);
		const std::vector<Tile> path = yard.FindPath(c.from, c.to);
		std::vector<Leg> legs = yard.Legs(path, 5.0f);
		if (c.crouched)
			legs = PaceLegs(legs, yard.Center(c.from), Pace::Sneak);
		const GoalPose pose = ChooseGoalPose(yard, clips, c.to, legs.back().heading, c.crouched);
		legs.back().end = pose.ground;
		legs.back().heading = pose.heading;
		std::vector<Vec3> route;
		for (const Tile& t : path)
			route.push_back(yard.Center(t));
		std::vector<LegReport> reports;
		if (!planner.Plan(player, legs, route, *pose.clip, walls, reports, &error))
		{
			std::printf("plan failed: %s\n", error.c_str());
			return 1;
		}
		for (size_t i = 0; i < legs.size(); ++i)
			std::printf("    leg %zu: %s, ends (%.2f, %.2f), %d tokens, contact %d frames %.0f cm\n", i,
				legs[i].vault ? (legs[i].standing_vault ? "standing vault" : "running vault")
							  : legs[i].last ? "last" : legs[i].sneak ? "sneak" : legs[i].walk ? "walk" : "run",
				legs[i].end.x, legs[i].end.z, reports[i].tokens, reports[i].contact_frames, reports[i].depth * 100);
		const bool standing_vault = std::any_of(legs.begin(), legs.end(), [](const Leg& l) { return l.standing_vault; });
		std::vector<ModelFrame> played;
		while (player.Busy())
		{
			player.Advance(1.0f / 30.0f);
			played.push_back(player.Frame());
		}
		const Contact contact = CheckContacts(played, walls);
		// The hands over the wall: the lowest a hand comes while above the wall's footprint.
		float hand = 10, peak = 0;
		int airborne = 0;
		for (size_t f = 0; f < played.size(); ++f)
		{
			for (int h : {13, 19}) // left and right hand
				for (const Obstacle& box : walls)
				{
					const Vec3& p = played[f].positions[h];
					if (p.x > box.min.x && p.x < box.max.x && p.z > box.min.z && p.z < box.max.z)
						hand = std::min(hand, p.y);
				}
			const bool l = std::min(played[f].positions[24].y, played[f].positions[25].y) < 0.06f;
			const bool r = std::min(played[f].positions[28].y, played[f].positions[29].y) < 0.06f;
			airborne += !l && !r;
			if (f > 0)
				peak = std::max(peak, std::hypot(played[f].positions[0].x - played[f - 1].positions[0].x,
										  played[f].positions[0].z - played[f - 1].positions[0].z) *
						30);
		}
		std::printf("  %-21s %s vault, %zu legs, %.1f s; wall contact %d frames, deepest %.0f cm (%s); hand over the "
					"wall %+.0f cm; hips peak %.2f m/s; airborne %d frames\n",
			c.name, standing_vault ? "standing" : "running", reports.size(), played.size() / 30.0f, contact.frames,
			contact.depth * 100, contact.joint >= 0 ? motion::Soma30JointNames()[contact.joint] : "-",
			hand < 10 ? (hand - 0.9f) * 100 : 0.0f, peak, airborne);
	}
	return 0;
}
