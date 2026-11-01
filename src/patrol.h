// An enemy patrolling an area in real time: he walks to a random spot in it, scans, turns toward the next spot if it
// lies well to the side, and walks on. Every move is a MotionBricks transition, like the soldier's.
#pragma once

#include <memory>
#include <random>
#include <vector>

#include "grid.h"
#include "motion_player.h"
#include "rig.h"

namespace tactics
{
	struct Vision
	{
		float range = 9.0f;          // meters
		float half_angle = 0.785f;   // radians either side of where the head faces
	};

	// Whether eyes looking along look_heading see a point: within the cone and with no cover in the way. Walls always
	// block the view; low walls only when the target is low (crouched, sneaking or in a low cover pose).
	bool Sees(const std::vector<Obstacle>& obstacles, const Vec3& eyes, float look_heading, const Vec3& target,
		bool target_low, const Vision& vision);
	// How far the eyes see along a ground direction, up to range: to the first wall, or the first wall or low wall when
	// looking for a low target.
	float SightDistance(const std::vector<Obstacle>& obstacles, const Vec3& eyes, float heading, float range,
		bool target_low);

	// The walkable tiles of a rectangle of the grid, corners included.
	std::vector<Tile> WalkableTiles(const Grid& grid, Tile from, Tile to);

	class Patrol
	{
	public:
		// spots: where he may go; he starts at start, facing start_heading.
		Patrol(const SomaSkeleton& skeleton, std::vector<Tile> spots, Tile start, float start_heading);

		// Back to the start: scanning at the starting spot.
		void Reset(const Grid& grid, const ClipSet& clips);
		// Plays the motion and plans the next step when one ends. Returns false with an error if a transition fails.
		bool Update(float dt, MovePlanner& planner, const Grid& grid, const ClipSet& clips,
			const std::vector<Obstacle>& obstacles, std::string* error);

		const MotionPlayer& Player() const { return *player_; }
		// Where the eyes are and the direction the face points on the ground.
		Vec3 Eyes() const;
		float LookHeading() const;

	private:
		enum class State
		{
			Scanning, // playing the look around at a spot
			Turning,  // turning in place toward the next spot
			Walking,  // on the way to the next spot
		};
		// Picks the next spot: a random one at least kMinStep tiles away.
		void ChooseNext();
		// Plays a clip once in place, starting from the current pose.
		bool PlayInPlace(const MotionClip& clip, MovePlanner& planner, const std::vector<Obstacle>& obstacles,
			std::string* error);

		const SomaSkeleton& skeleton_;
		std::vector<Tile> spots_;
		Tile start_;
		float start_heading_ = 0;
		std::unique_ptr<MotionPlayer> player_;
		State state_ = State::Scanning;
		Tile at_;                // the spot reached last
		Tile next_;              // the spot he heads for next
		float remaining_ = 0;    // seconds of the clip playing in place that are left once the queue is empty
		std::mt19937 random_{std::random_device{}()}; // seeded once, so every round goes differently
		std::vector<LegReport> reports_;
	};
}
