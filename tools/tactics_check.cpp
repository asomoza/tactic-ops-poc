// Headless checks of the rig conversions and of planning a move with MotionBricks.
//
// Usage: tactics_check [model_dir]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <tuple>

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
	int driven = 0;
	for (int k : skeleton.model_of)
		driven += k >= 0;
	std::printf("skeleton: %zu joints, %d follow the model\n", skeleton.names.size(), driven);

	// A placed clip frame as a mesh pose should put the joints where the clip has them, up to the difference between
	// the clip's and the mesh's bone lengths and the mesh joints that keep their rest rotation.
	const Placement placement = PlaceClipFrame(clips.run, 40, {3, 0, 4}, 1.0f);
	const ModelFrame model = ModelFrameFromClip(clips.run, 40, placement);
	std::vector<Transform> model_world;
	PoseWorld(skeleton, PoseFromModel(skeleton, model), model_world);
	float position = 0;
	for (int k = 0; k < kModelJoints; ++k)
	{
		const int j = skeleton.model_joints[k];
		if (skeleton.model_of[j] >= 0)
			position = std::max(position, (model_world[j].translation - model.positions[k]).Length());
	}
	std::printf("clip frames: stand %d, crouch %d, run %d, walk %d; mesh vs clip joints %.3f m\n", clips.stand.Frames(),
		clips.crouch.Frames(), clips.run.Frames(), clips.walk.Frames(), position);
	std::printf("placed heading %.3f (wanted 1.000), hips ground (%.2f, %.2f) (wanted 3, 4)\n", FrameHeading(model),
		model.positions[0].x, model.positions[0].z);

	// Mirrored clips: the mesh posed from the mirrored rotations should put the joints at the mirrored positions as
	// closely as it does for the original, and a turn should end turned the other way.
	auto mesh_error = [&](const MotionClip& clip) {
		float worst = 0;
		for (int f = 0; f < clip.Frames(); f += 5)
		{
			const ModelFrame& frame = clip.frames[f];
			std::vector<Transform> world;
			PoseWorld(skeleton, PoseFromModel(skeleton, frame), world);
			for (int k = 0; k < kModelJoints; ++k)
			{
				const int j = skeleton.model_joints[k];
				if (skeleton.model_of[j] >= 0)
					worst = std::max(worst, (world[j].translation - frame.positions[k]).Length());
			}
		}
		return worst;
	};
	auto turned = [](const MotionClip& clip) {
		return WrapAngle(FrameHeading(clip.frames.back()) - FrameHeading(clip.frames.front())) * 57.29578f;
	};
	for (const auto& [name, original, mirrored] :
		{std::tuple{"turn 90", &clips.turn_left_90, &clips.turn_right_90},
			std::tuple{"turn 180", &clips.turn_left_180, &clips.turn_right_180},
			std::tuple{"look around", &clips.look_around, &clips.look_around_mirrored}})
		std::printf("%s: turns %+.0f deg, mirrored %+.0f deg; mesh vs clip joints %.3f m, mirrored %.3f m\n", name,
			turned(*original), turned(*mirrored), mesh_error(*original), mesh_error(*mirrored));

	// Each idle looping twice: the largest joint step between frames should be no larger at the loop point than
	// within the clip.
	for (const auto& [name, clip] : {std::pair{"stand", &clips.stand}, std::pair{"crouch", &clips.crouch},
			 std::pair{"wall facing", &clips.wall_facing}, std::pair{"wall back", &clips.wall_back},
			 std::pair{"low facing", &clips.low_facing}, std::pair{"low back", &clips.low_back}})
	{
		MotionPlayer looping(skeleton);
		looping.SetLoop(clip, clip->loop_start, {});
		float within = 0, seam = 0;
		ModelFrame previous = looping.Frame();
		const int period = clip->Frames() - clip->loop_start;
		for (int f = 0; f < 2 * period; ++f)
		{
			looping.Advance(1.0f / 30.0f);
			float step = 0;
			for (int k = 0; k < kModelJoints; ++k)
				step = std::max(step, (looping.Frame().positions[k] - previous.positions[k]).Length());
			const int at = (f + 1) % period;
			float& worst = at < 20 || at > period - 20 ? seam : within;
			worst = std::max(worst, step);
			previous = looping.Frame();
		}
		std::printf("%s loop: largest joint step %.1f cm within the clip, %.1f cm around the loop point\n", name,
			within * 100, seam * 100);
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
	grid.Set({4, 2}, Cell::HalfCover);
	const Tile start{2, 2}, goal{8, 5};
	grid.Set({9, 5}, Cell::HalfCover);
	const std::vector<Tile> path = grid.FindPath(start, goal);
	std::vector<Leg> legs = grid.Legs(path, std::getenv("MAX_LEG") ? float(std::atof(std::getenv("MAX_LEG"))) : 5.0f);
	std::printf("path of %zu tiles, %zu legs\n", path.size(), legs.size());

	MotionPlayer player(skeleton);
	player.SetLoop(&clips.stand, 0, PlaceClipFrame(clips.stand, 0, grid.Center(start), 0));
	player.Advance(0.5f);
	const float run_speed = std::getenv("RUN_SPEED") ? std::atof(std::getenv("RUN_SPEED")) : 3.5f;
	MovePlanner planner(*bricks, clips, grid, run_speed);
	const std::vector<Obstacle> obstacles = grid.Obstacles();
	std::vector<LegReport> reports;
	std::vector<Vec3> route;
	for (const Tile& t : path)
		route.push_back(grid.Center(t));
	if (!planner.Plan(player, legs, route, clips.crouch, obstacles, reports, &error))
	{
		std::printf("plan failed: %s\n", error.c_str());
		return 1;
	}
	for (size_t i = 0; i < reports.size(); ++i)
		std::printf("leg %zu%s: %.2f m, %d frames (%d tokens), %.1f ms; ends at (%.2f, %.2f) heading %.2f; contacts %d "
			"frames, %.0f cm\n", i + 1, legs[i].detour ? " (detour)" : "", reports[i].distance, reports[i].frames,
			reports[i].tokens, reports[i].milliseconds, legs[i].end.x, legs[i].end.z, legs[i].heading,
			reports[i].contact_frames, reports[i].depth * 100);
	// Play everything and check where the character stops.
	int frames = 0;
	float max_step = 0;
	Vec3 previous = player.Frame().positions[0];
	while (player.Busy())
	{
		player.Advance(1.0f / 30.0f);
		const Vec3 hips = player.Frame().positions[0];
		max_step = std::max(max_step, (Vec3{hips.x, 0, hips.z} - Vec3{previous.x, 0, previous.z}).Length() * 30);
		previous = hips;
		++frames;
	}
	const ModelFrame& last = player.Frame();
	std::printf("played %d frames (%.1f s); fastest hips speed %.2f m/s; stopped at (%.2f, %.2f), hips height %.2f, "
		"heading %.2f\n",
		frames, frames / 30.0f, max_step, last.positions[0].x, last.positions[0].z, last.positions[0].y,
		FrameHeading(last));
	std::printf("goal (%.2f, %.2f), heading %.2f\n", grid.Center(goal).x, grid.Center(goal).z, legs.back().heading);
	return 0;
}
