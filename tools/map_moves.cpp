// Plans many moves on the game's map, each from where the previous one stopped, and reports the legs, the splits
// at path tiles and any contact with cover that remains.
//
// Usage: map_moves [model_dir]

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

	// The game's map.
	Grid grid(12, 12, 1.5f);
	BuildYard(grid);
	const std::vector<Obstacle> obstacles = grid.Obstacles();

	MotionPlayer player(skeleton);
	Tile at{2, 2};
	player.SetLoop(&clips.stand, 0, PlaceClipFrame(clips.stand, 0, grid.Center(at), 0));
	player.Advance(0.2f);
	MovePlanner planner(*bricks, clips, grid, 3.0f);
	// Moves through the tight spots between walls and low walls, then a few open ones.
	// The last three cross the low wall at (7, 6) and back.
	const Tile goals[] = {{6, 5}, {8, 7}, {6, 5}, {7, 7}, {4, 6}, {6, 8}, {8, 8}, {7, 5}, {5, 5}, {8, 7}, {4, 7},
		{10, 8}, {3, 3}, {9, 4}, {6, 9}, {7, 8}, {7, 4}, {7, 8}};
	std::printf("cover reach: wall facing %.2f m, wall back %.2f m, low facing %.2f m, low back %.2f m\n",
		clips.wall_facing.reach, clips.wall_back.reach, clips.low_facing.reach, clips.low_back.reach);
	int clean = 0, total = 0;
	for (const Tile& goal : goals)
	{
		const std::vector<Tile> path = grid.FindPath(at, goal);
		std::vector<Leg> legs = grid.Legs(path, 5.0f);
		std::vector<Vec3> route;
		for (const Tile& t : path)
			route.push_back(grid.Center(t));
		// Every other move asks to face away from the travel direction, which puts the back to the cover.
		const float facing = total % 2 ? legs.back().heading + 3.14159265f : legs.back().heading;
		const GoalPose pose = ChooseGoalPose(grid, clips, goal, facing);
		legs.back().end = pose.ground;
		legs.back().heading = pose.heading;
		std::vector<LegReport> reports;
		if (!planner.Plan(player, legs, route, *pose.clip, obstacles, reports, &error))
		{
			std::printf("plan failed: %s\n", error.c_str());
			return 1;
		}
		int splits = 0, contacts = 0, vaults = 0;
		float depth = 0, shortest = 1e9f;
		for (size_t i = 0; i < reports.size(); ++i)
		{
			if (reports[i].contact_frames)
				std::printf("    leg %zu%s ends (%.2f, %.2f) heading %.2f, %s: %d contact frames, %.0f cm\n", i + 1,
					legs[i].detour ? " (split)" : "", legs[i].end.x, legs[i].end.z, legs[i].heading,
					legs[i].last ? "last" : legs[i].walk ? "walk" : "run", reports[i].contact_frames, reports[i].depth * 100);
			splits += legs[i].detour;
			vaults += legs[i].vault;
			contacts += reports[i].contact_frames;
			depth = std::max(depth, reports[i].depth);
			shortest = std::min(shortest, reports[i].distance);
		}
		// The goal idle itself, placed against its cover: the deepest contact, and the gap left to the cover's face.
		std::vector<ModelFrame> idle;
		const Placement placement = PlaceClipFrame(*pose.clip, pose.clip->loop_start, pose.ground, pose.heading);
		for (int f = pose.clip->loop_start; f < pose.clip->Frames(); ++f)
			idle.push_back(ModelFrameFromClip(*pose.clip, f, placement));
		const Contact idle_contact = CheckContacts(idle, obstacles);
		std::printf("(%d,%d) -> (%d,%d): %zu path tiles, %zu legs (%d splits, %d vaults), shortest leg %.2f m; contact %d frames, "
			"deepest %.0f cm; ends in %s, which touches cover %.1f cm deep\n",
			at.x, at.y, goal.x, goal.y, path.size(), reports.size(), splits, vaults, shortest, contacts, depth * 100,
			pose.clip->name.c_str(), idle_contact.depth * 100);
		{
			std::vector<ModelFrame> queued;
			MotionPlayer copy = player;
			while (copy.Busy())
			{
				copy.Advance(1.0f / 30.0f);
				queued.push_back(copy.Frame());
			}
			const Contact c = CheckContacts(queued, obstacles);
			if (c.frames)
			{
				const Obstacle& b = obstacles[c.obstacle];
				std::printf("    deepest: %.0f cm, joint %d (%s) at frame %d of %zu, hips (%.2f, %.2f, %.2f), box x %.2f..%.2f "
					"z %.2f..%.2f h %.1f\n",
					c.depth * 100, c.joint, motion::Soma30JointNames()[c.joint], c.frame, queued.size(), c.hips.x, c.hips.y, c.hips.z,
					b.min.x, b.max.x, b.min.z, b.max.z, b.max.y);
				const ModelFrame& f = queued[c.frame];
				const Vec3 j = f.positions[c.joint];
				std::printf("    joint at (%.2f, %.2f, %.2f)\n", j.x, j.y, j.z);
			}
		}
		clean += depth <= 0.03f && idle_contact.depth <= 0.03f;
		++total;
		while (player.Busy())
			player.Advance(1.0f / 30.0f);
		at = goal;
	}
	std::printf("%d of %d moves clear of cover, moving and at the goal (contacts under 3 cm; vaults touch their wall on "
		"purpose)\n", clean, total);
	return 0;
}
