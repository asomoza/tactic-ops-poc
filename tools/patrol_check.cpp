// Runs the game's two patrols headless for a few minutes: logs every clip they play, checks the motion against the
// walls, and tests the vision on fixed cases and from a few soldier spots, standing and low.
//
// Usage: patrol_check [model_dir]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include "grid.h"
#include "motion_player.h"
#include "patrol.h"
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

	// The game's yard and patrol.
	Grid grid(12, 12, 1.5f);
	BuildYard(grid);
	const std::vector<Obstacle> obstacles = grid.Obstacles();
	// The vision rules on fixed cases: a soldier at (8, 5) behind the low wall at (7, 6)-(8, 6), seen from (8, 8)
	// across the wall and from (11, 5) on his side; and one behind the wall at (6, 3)-(6, 4), seen from (8, 4).
	{
		const Vision v;
		const Vec3 across = grid.Center({8, 8}) + Vec3{0, 1.65f, 0}, same_side = grid.Center({11, 5}) + Vec3{0, 1.65f, 0};
		const Vec3 standing = grid.Center({8, 5}) + Vec3{0, 1.6f, 0}, low = grid.Center({8, 5}) + Vec3{0, 1.05f, 0};
		const float north = 3.14159265f, south = 0;
		std::printf("vision: across the low wall, standing %s, low %s; same side, low %s; behind the wall %s; "
					"looking away %s\n",
			Sees(obstacles, across, north, standing, false, v) ? "seen" : "hidden",
			Sees(obstacles, across, north, low, true, v) ? "seen" : "hidden",
			Sees(obstacles, same_side, -1.5708f, low, true, v) ? "seen" : "hidden",
			Sees(obstacles, grid.Center({8, 4}) + Vec3{0, 1.65f, 0}, -1.5708f, grid.Center({5, 3}) + Vec3{0, 1.6f, 0},
				false, v)
				? "seen"
				: "hidden",
			Sees(obstacles, across, south, standing, false, v) ? "seen" : "hidden");
	}
	// The game's two patrols.
	MovePlanner planner(*bricks, clips, grid, 3.0f);
	const float south = 3.14159265f;
	const std::vector<Tile> yard = WalkableTiles(grid, {0, 0}, {11, 11});
	Patrol patrols[] = {Patrol(skeleton, yard, {1, 10}, south), Patrol(skeleton, yard, {10, 10}, south)};
	const char* names[] = {"west", "east"};
	struct Track
	{
		const MotionClip* clip = nullptr;
		std::vector<ModelFrame> played;
		float peak = 0;
		int airborne = 0;
	} tracks[2];
	for (Patrol& patrol : patrols)
		patrol.Reset(grid, clips);

	const float dt = 1.0f / 30.0f;
	const Vision vision;
	// Soldier spots, seen by either enemy: the start, and a few on the way north.
	struct Spot
	{
		const char* name;
		Vec3 head;
		bool low;
		int seen = 0;
	} spots[] = {{"the start (7, 2), back to the wall", grid.Center({7, 2}) + Vec3{0, 1.6f, 0}, false},
		{"open ground (8, 4), standing", grid.Center({8, 4}) + Vec3{0, 1.6f, 0}, false},
		{"open ground (8, 4), low", grid.Center({8, 4}) + Vec3{0, 1.05f, 0}, true},
		{"behind the low wall at (8, 5), standing", grid.Center({8, 5}) + Vec3{0, 1.6f, 0}, false},
		{"behind the low wall at (8, 5), low", grid.Center({8, 5}) + Vec3{0, 1.05f, 0}, true},
		{"behind the wall at (4, 7), standing", grid.Center({4, 7}) + Vec3{0, 1.6f, 0}, false}};
	const int steps = int(180 / dt);
	for (int step = 0; step < steps; ++step)
	{
		for (int i = 0; i < 2; ++i)
		{
			Patrol& patrol = patrols[i];
			Track& track = tracks[i];
			if (!patrol.Update(dt, planner, grid, clips, obstacles, &error))
			{
				std::printf("%s patrol failed at %.1f s: %s\n", names[i], step * dt, error.c_str());
				return 1;
			}
			const ModelFrame& f = patrol.Player().Frame();
			if (!patrol.Player().Busy() && patrol.Player().LoopClip() != track.clip)
			{
				track.clip = patrol.Player().LoopClip();
				std::printf("  %6.1f s: %s %-24s at tile (%.1f, %.1f)\n", step * dt, names[i], track.clip->name.c_str(),
					f.positions[0].x / 1.5f - 0.5f, f.positions[0].z / 1.5f - 0.5f);
			}
			if (!track.played.empty())
				track.peak = std::max(track.peak, std::hypot(f.positions[0].x - track.played.back().positions[0].x,
													  f.positions[0].z - track.played.back().positions[0].z) /
						dt);
			const bool l = std::min(f.positions[24].y, f.positions[25].y) < 0.06f;
			const bool r = std::min(f.positions[28].y, f.positions[29].y) < 0.06f;
			track.airborne += !l && !r;
			track.played.push_back(f);
		}
		for (Spot& s : spots)
			s.seen += Sees(obstacles, patrols[0].Eyes(), patrols[0].LookHeading(), s.head, s.low, vision) ||
				Sees(obstacles, patrols[1].Eyes(), patrols[1].LookHeading(), s.head, s.low, vision);
	}
	for (int i = 0; i < 2; ++i)
	{
		const Contact contact = CheckContacts(tracks[i].played, obstacles);
		std::printf("%s patrol, %.0f s: walls touched in %d frames, deepest %.0f cm; hips peak %.2f m/s; airborne %d "
					"frames\n",
			names[i], steps * dt, contact.frames, contact.depth * 100, tracks[i].peak, tracks[i].airborne);
	}
	for (const Spot& s : spots)
		std::printf("  seen at %-40s %5.1f%% of the time\n", s.name, 100.0f * s.seen / steps);
	return 0;
}
