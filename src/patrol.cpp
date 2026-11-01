#include "patrol.h"

#include <algorithm>
#include <cmath>

namespace tactics
{
	namespace
	{
		constexpr float kFrameTime = 1.0f / 30.0f;
		// SOMA30 joints.
		constexpr int kHead = 6, kLeftEye = 8, kRightEye = 9;
		// Covers at least this tall block the view of a standing target.
		constexpr float kWallHeight = 1.5f;
		// The shortest walk between two spots, in tiles.
		constexpr float kMinStep = 3.0f;

		// Where the ground segment from a to b enters the box's footprint, as a fraction of the segment (slab test on x
		// and z), or a negative value if it misses.
		float Entry(const Vec3& a, const Vec3& b, const Obstacle& box)
		{
			float t0 = 0, t1 = 1;
			const float start[2] = {a.x, a.z}, delta[2] = {b.x - a.x, b.z - a.z};
			const float low[2] = {box.min.x, box.min.z}, high[2] = {box.max.x, box.max.z};
			for (int axis = 0; axis < 2; ++axis)
			{
				if (std::abs(delta[axis]) < 1e-6f)
				{
					if (start[axis] < low[axis] || start[axis] > high[axis])
						return -1;
					continue;
				}
				float enter = (low[axis] - start[axis]) / delta[axis], leave = (high[axis] - start[axis]) / delta[axis];
				if (enter > leave)
					std::swap(enter, leave);
				t0 = std::max(t0, enter);
				t1 = std::min(t1, leave);
				if (t0 > t1)
					return -1;
			}
			return t0;
		}

		bool Crosses(const Vec3& a, const Vec3& b, const Obstacle& box)
		{
			return Entry(a, b, box) >= 0;
		}
	}

	bool Sees(const std::vector<Obstacle>& obstacles, const Vec3& eyes, float look_heading, const Vec3& target,
		bool target_low, const Vision& vision)
	{
		const Vec3 to{target.x - eyes.x, 0, target.z - eyes.z};
		if (to.Length() > vision.range || std::abs(WrapAngle(HeadingOf(to) - look_heading)) > vision.half_angle)
			return false;
		for (const Obstacle& box : obstacles)
			if ((box.max.y >= kWallHeight || target_low) && Crosses(eyes, target, box))
				return false;
		return true;
	}

	std::vector<Tile> WalkableTiles(const Grid& grid, Tile from, Tile to)
	{
		std::vector<Tile> tiles;
		for (int y = from.y; y <= to.y; ++y)
			for (int x = from.x; x <= to.x; ++x)
				if (grid.Walkable({x, y}))
					tiles.push_back({x, y});
		return tiles;
	}

	float SightDistance(const std::vector<Obstacle>& obstacles, const Vec3& eyes, float heading, float range,
		bool target_low)
	{
		const Vec3 end = eyes + Vec3{std::sin(heading), 0, std::cos(heading)} * range;
		float nearest = 1;
		for (const Obstacle& box : obstacles)
			if (box.max.y >= kWallHeight || target_low)
			{
				const float t = Entry(eyes, end, box);
				if (t >= 0)
					nearest = std::min(nearest, t);
			}
		return nearest * range;
	}

	Patrol::Patrol(const SomaSkeleton& skeleton, std::vector<Tile> spots, Tile start, float start_heading)
		: skeleton_(skeleton), spots_(std::move(spots)), start_(start), start_heading_(start_heading)
	{
	}

	void Patrol::Reset(const Grid& grid, const ClipSet& clips)
	{
		player_ = std::make_unique<MotionPlayer>(skeleton_);
		const MotionClip& scan = clips.look_around;
		player_->SetLoop(&scan, 0, PlaceClipFrame(scan, 0, grid.Center(start_), start_heading_));
		state_ = State::Scanning;
		at_ = start_;
		ChooseNext();
		remaining_ = scan.Frames() * kFrameTime;
	}

	void Patrol::ChooseNext()
	{
		std::vector<Tile> distant;
		for (const Tile& t : spots_)
			if (std::hypot(float(t.x - at_.x), float(t.y - at_.y)) >= kMinStep)
				distant.push_back(t);
		const std::vector<Tile>& from = distant.empty() ? spots_ : distant;
		next_ = from[std::uniform_int_distribution<size_t>(0, from.size() - 1)(random_)];
	}

	bool Patrol::PlayInPlace(const MotionClip& clip, MovePlanner& planner, const std::vector<Obstacle>& obstacles,
		std::string* error)
	{
		const ModelFrame& now = player_->Frame();
		Leg leg;
		leg.end = {now.positions[0].x, 0, now.positions[0].z};
		leg.heading = FrameHeading(now);
		leg.last = true;
		leg.min_tokens = 6;
		leg.max_tokens = 8;
		std::vector<Leg> legs{leg};
		if (!planner.Plan(*player_, legs, {leg.end}, clip, obstacles, reports_, error))
			return false;
		// The clip loops from its fifth frame once the transition into it has played: one pass is left.
		remaining_ = (clip.Frames() - 4) * kFrameTime;
		return true;
	}

	bool Patrol::Update(float dt, MovePlanner& planner, const Grid& grid, const ClipSet& clips,
		const std::vector<Obstacle>& obstacles, std::string* error)
	{
		player_->Advance(dt);
		if (player_->Busy())
			return true;
		if (state_ == State::Walking)
		{
			// Arrived: the scan plays once.
			state_ = State::Scanning;
			remaining_ = (player_->LoopClip()->Frames() - 4) * kFrameTime;
		}
		remaining_ -= dt;
		if (remaining_ > 0)
			return true;

		const ModelFrame& now = player_->Frame();
		const Vec3 ground{now.positions[0].x, 0, now.positions[0].z};
		const float turn = WrapAngle(HeadingOf(grid.Center(next_) - ground) - FrameHeading(now));
		if (state_ == State::Scanning && std::abs(turn) > 0.785f)
		{
			// Positive turns are to the left, as the clips were generated.
			const bool left = turn > 0;
			const MotionClip& clip = std::abs(turn) > 2.36f ? (left ? clips.turn_left_180 : clips.turn_right_180)
															: (left ? clips.turn_left_90 : clips.turn_right_90);
			state_ = State::Turning;
			return PlayInPlace(clip, planner, obstacles, error);
		}

		const std::vector<Tile> path = grid.FindPath(at_, next_);
		if (path.size() < 2)
		{
			// No way there: pick another spot and scan once more.
			ChooseNext();
			state_ = State::Scanning;
			remaining_ = 1.0f;
			return true;
		}
		std::vector<Leg> legs = PaceLegs(grid.Legs(path, 5.0f), grid.Center(path.front()), Pace::Walk);
		std::vector<Vec3> route;
		for (const Tile& t : path)
			route.push_back(grid.Center(t));
		// The scan, as generated or mirrored, so that he looks first to one side or the other.
		const MotionClip& scan = std::bernoulli_distribution(0.5)(random_) ? clips.look_around : clips.look_around_mirrored;
		if (!planner.Plan(*player_, legs, route, scan, obstacles, reports_, error))
			return false;
		at_ = next_;
		ChooseNext();
		state_ = State::Walking;
		return true;
	}

	Vec3 Patrol::Eyes() const
	{
		const ModelFrame& f = player_->Frame();
		return (f.positions[kLeftEye] + f.positions[kRightEye]) * 0.5f;
	}

	float Patrol::LookHeading() const
	{
		const ModelFrame& f = player_->Frame();
		return HeadingOf(Eyes() - f.positions[kHead]);
	}
}
