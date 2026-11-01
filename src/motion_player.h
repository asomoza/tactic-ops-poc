// Plays a character's motion at 30 FPS: generated transitions queued frame by frame, and a looping clip when the
// queue is empty, its end blended into its loop start. Plans a move as one MotionBricks transition per leg, each
// continuing from the previous one.
#pragma once

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "clip.h"
#include "grid.h"
#include "motion/motionbricks.h"
#include "rig.h"

namespace tactics
{
	struct PlayedFrame
	{
		ModelFrame model;
		RigPose pose;
	};

	// The clips a character uses.
	struct ClipSet
	{
		MotionClip stand;   // idle, standing
		MotionClip crouch;  // idle, crouched on open ground
		MotionClip run;     // forward run; the targets of intermediate legs
		int run_frame = 60; // the run frame whose four poses, made in place, are used as targets
		MotionClip walk;    // forward walk; the targets of walking legs
		int walk_frame = 75;
		// Idles next to cover: facing a wall or low wall, or with the back to it.
		MotionClip wall_facing, wall_back, low_facing, low_back;
		// Running over a low wall: a move arrives on vault_from, a tile in front of the wall, and the clip plays until
		// vault_to, just after landing.
		MotionClip vault;
		int vault_from = 50, vault_to = 82;
		float vault_heading = 0; // the clip's direction of travel from vault_from to vault_to
		float vault_plant = 0;   // how far ahead of the hips at vault_from the hand plants, along that direction
		// Crossing a low wall from standing; its crossing is the vault's frames 58 to 90, from frame 30. A move arrives
		// on vault_stand_from, standing just before the first step, and the clip plays until vault_stand_to, landed and
		// standing.
		MotionClip vault_stand;
		int vault_stand_from = 10, vault_stand_to = 90;
		float vault_stand_heading = 0, vault_stand_plant = 0;
		MotionClip sneak; // forward crouched walk; the targets of crouched legs
		int sneak_frame = 75;
		// Turning in place, to the left as generated and mirrored to the right.
		MotionClip turn_left_90, turn_right_90, turn_left_180, turn_right_180;
		// Standing still and scanning, as generated and mirrored.
		MotionClip look_around, look_around_mirrored;
	};

	// Load the clips from a folder of Kinetic Artisan motions (assets/motions/kimodo).
	bool LoadClipSet(const std::string& dir, ClipSet& clips, std::string* error);

	// How a move ends: the idle clip, where its hips stand on the ground and where it faces.
	struct GoalPose
	{
		const MotionClip* clip = nullptr;
		Vec3 ground;
		float heading = 0;
	};

	enum class Pace
	{
		Sneak, // crouched, about 0.8 m/s
		Walk,  // about 1.2 m/s
	};
	// A move's legs split into short pieces at a slow pace, each ending in a pose of that pace except the last, which
	// keeps its goal. start is the ground point the move starts from. A vault becomes a standing vault at the end of
	// its leg's last piece.
	std::vector<Leg> PaceLegs(const std::vector<Leg>& legs, const Vec3& start, Pace pace);

	// The goal pose on a tile for a wanted facing: on open ground, standing or crouched. Next to cover, the pose faces
	// the cover when the facing is within 90 degrees of it and turns its back to it otherwise, placed so that the body
	// touches the cover's face, or keeps the pose's gap to it.
	GoalPose ChooseGoalPose(const Grid& grid, const ClipSet& clips, Tile goal, float heading, bool crouched = false);

	struct LegReport
	{
		int tokens = 0;
		int frames = 0;
		float distance = 0;
		double milliseconds = 0;
		bool detour = false;     // ends at a waypoint added to steer around an obstacle
		int contact_frames = 0;  // frames in which a body point is inside an obstacle
		float depth = 0;         // the deepest such contact, in meters
	};

	class MotionPlayer
	{
	public:
		MotionPlayer(const SomaSkeleton& skeleton) : skeleton_(skeleton) {}

		// Loop a clip from start_frame with a placement once the queue is empty.
		void SetLoop(const MotionClip* clip, int start_frame, const Placement& placement);
		void Queue(const ModelFrame& frame);
		bool Busy() const { return !queue_.empty(); }
		void Advance(float seconds);
		const RigPose& Pose() const { return current_.pose; }
		const ModelFrame& Frame() const { return current_.model; }
		// The last four frames played or queued: the context of the next transition.
		std::array<ModelFrame, 4> Tail() const;

		const MotionClip* LoopClip() const { return loop_; }
		const Placement& LoopPlacement() const { return loop_placement_; }

	private:
		PlayedFrame FromLoop();
		void Push(const PlayedFrame& frame);

		const SomaSkeleton& skeleton_;
		std::deque<ModelFrame> queue_;
		std::deque<ModelFrame> history_; // the last four frames played
		PlayedFrame current_;
		const MotionClip* loop_ = nullptr;
		int loop_frame_ = 0;
		Placement loop_placement_;
		float clock_ = 0;
	};

	// Turns paths into motion with MotionBricks.
	class MovePlanner
	{
	public:
		MovePlanner(motion::MotionBricks& bricks, const ClipSet& clips, const Grid& grid, float run_speed)
			: bricks_(bricks), clips_(clips), grid_(grid), run_speed_(run_speed)
		{
		}

		// Queue the transitions of a move on the player, then loop the goal clip where the last leg ends. When a leg's
		// body passes through an obstacle, the leg is split at the path tile nearest the contact and generated again
		// as two legs; legs ends up with those splits. Returns false with an error if a transition fails; nothing is
		// queued then.
		// path holds the ground points of the path's tiles, which the legs index.
		bool Plan(MotionPlayer& player, std::vector<Leg>& legs, const std::vector<Vec3>& path, const MotionClip& goal_clip,
			const std::vector<Obstacle>& obstacles, std::vector<LegReport>& reports, std::string* error);

	private:
		std::array<ModelFrame, 4> Target(const Leg& leg, const MotionClip& goal_clip, const MotionClip*& clip,
			Placement& placement) const;
		// The frames from the end of `context` to the target: the transition without its first frame, then the target
		// frames after the first.
		bool Generate(const std::array<ModelFrame, 4>& context, const std::array<ModelFrame, 4>& target,
			std::vector<ModelFrame>& frames, LegReport& report, std::string* error, int min_tokens = 0,
			int max_tokens = 0);

		motion::MotionBricks& bricks_;
		const ClipSet& clips_;
		const Grid& grid_;
		float run_speed_;

	public:
		// The transition lengths the model may choose, in tokens of 4 frames (6 to 16).
		int min_tokens = 8, max_tokens = 14;
		// The speed of walking and sneaking targets, in meters per second.
		float walk_speed = 1.2f;
		float sneak_speed = 0.8f;
	};

	// Body points inside obstacles: how many frames touch one, the deepest contact, and where the hips were in the
	// frame with the deepest contact, with the obstacle it touched.
	struct Contact
	{
		int frames = 0;
		float depth = 0;
		Vec3 hips;
		int obstacle = -1;
		int joint = -1;  // the SOMA30 joint of the deepest contact
		int frame = -1;  // the frame of the deepest contact
	};
	Contact CheckContacts(const std::vector<ModelFrame>& frames, const std::vector<Obstacle>& obstacles);
}
