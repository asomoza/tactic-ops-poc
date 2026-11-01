#include "motion_player.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace tactics
{
	namespace
	{
		constexpr float kFrameTime = 1.0f / 30.0f;
		constexpr float kPi = 3.14159265f;
		constexpr int kLoopBlend = 15; // frames over which a looping clip's end blends into its loop start
		constexpr int kRightHand = 19; // SOMA30 joint
		constexpr float kDetourDepth = 0.03f; // contacts deeper than this get a detour, in meters

		// The SOMA30 joints checked against obstacles, with their radius in meters: hips, chest, head, hands,
		// shins and feet.
		constexpr std::pair<int, float> kBodyPoints[] = {
			{0, 0.14f}, {3, 0.16f}, {6, 0.11f}, {13, 0.05f}, {19, 0.05f}, {23, 0.07f}, {27, 0.07f}, {24, 0.06f},
			{28, 0.06f}, {25, 0.05f}, {29, 0.05f},
		};

		void Flatten(const std::array<ModelFrame, 4>& frames, std::vector<float>& joints, std::vector<float>& rotations)
		{
			joints.resize(4 * kModelJoints * 3);
			rotations.resize(4 * kModelJoints * 9);
			for (int f = 0; f < 4; ++f)
				for (int k = 0; k < kModelJoints; ++k)
				{
					const Vec3& p = frames[f].positions[k];
					float* joint = &joints[(f * kModelJoints + k) * 3];
					joint[0] = p.x;
					joint[1] = p.y;
					joint[2] = p.z;
					std::copy(frames[f].rotations[k].m.begin(), frames[f].rotations[k].m.end(),
						&rotations[(f * kModelJoints + k) * 9]);
				}
		}

		ModelFrame FromTransition(const motion::Transition& t, int frame)
		{
			ModelFrame out;
			for (int k = 0; k < kModelJoints; ++k)
			{
				const float* p = &t.joints[(size_t(frame) * kModelJoints + k) * 3];
				out.positions[k] = {p[0], p[1], p[2]};
				const float* r = &t.global_rotations[(size_t(frame) * kModelJoints + k) * 9];
				std::copy(r, r + 9, out.rotations[k].m.begin());
			}
			return out;
		}

		std::array<ModelFrame, 4> LastFour(const std::array<ModelFrame, 4>& before, const std::vector<ModelFrame>& added)
		{
			std::vector<ModelFrame> all(before.begin(), before.end());
			all.insert(all.end(), added.begin(), added.end());
			std::array<ModelFrame, 4> tail;
			for (int i = 0; i < 4; ++i)
				tail[i] = all[all.size() - 4 + i];
			return tail;
		}

		Vec3 Ground(const Vec3& p)
		{
			return {p.x, 0, p.z};
		}

		// How far the body spheres reach from the hips' ground point along the clip's facing (direction 1) or against
		// it (-1), at most over the settled frames, measured in the frame the clip is placed by.
		float Reach(const MotionClip& clip, float direction)
		{
			const ModelFrame& start = clip.frames[clip.loop_start];
			const float heading = FrameHeading(start);
			const Vec3 forward = Vec3{std::sin(heading), 0, std::cos(heading)} * direction;
			float reach = 0;
			for (int f = clip.loop_start; f < clip.Frames(); ++f)
				for (const auto& [joint, radius] : kBodyPoints)
				{
					const Vec3 d = Ground(clip.frames[f].positions[joint]) - Ground(start.positions[0]);
					reach = std::max(reach, d.x * forward.x + d.z * forward.z + radius);
				}
			return reach;
		}
	}

	bool LoadClipSet(const std::string& dir, ClipSet& clips, std::string* error)
	{
		if (!LoadMotionClip(dir + "/stand_idle.npz", clips.stand, error) ||
			!LoadMotionClip(dir + "/crouch_idle.npz", clips.crouch, error) ||
			!LoadMotionClip(dir + "/run_loop.npz", clips.run, error) || !LoadMotionClip(dir + "/walk.npz", clips.walk, error))
			return false;
		if (!LoadMotionClip(dir + "/wall_facing.npz", clips.wall_facing, error) ||
			!LoadMotionClip(dir + "/wall_back.npz", clips.wall_back, error) ||
			!LoadMotionClip(dir + "/low_facing.npz", clips.low_facing, error) ||
			!LoadMotionClip(dir + "/low_back.npz", clips.low_back, error) ||
			!LoadMotionClip(dir + "/vault.npz", clips.vault, error) ||
			!LoadMotionClip(dir + "/vault_stand.npz", clips.vault_stand, error) ||
			!LoadMotionClip(dir + "/sneak.npz", clips.sneak, error) ||
			!LoadMotionClip(dir + "/turn_90.npz", clips.turn_left_90, error) ||
			!LoadMotionClip(dir + "/turn_180.npz", clips.turn_left_180, error) ||
			!LoadMotionClip(dir + "/look_around.npz", clips.look_around, error))
			return false;
		clips.turn_right_90 = MirrorClip(clips.turn_left_90);
		clips.turn_right_180 = MirrorClip(clips.turn_left_180);
		clips.look_around_mirrored = MirrorClip(clips.look_around);
		// Where each clip has settled: the crouches take a second to go down; the wall lean and the stop at the wall
		// take three.
		clips.crouch.loop_start = 30;
		clips.wall_facing.loop_start = 90;
		clips.wall_back.loop_start = 90;
		clips.low_facing.loop_start = 30;
		clips.low_back.loop_start = 30;
		clips.wall_facing.reach = Reach(clips.wall_facing, 1);
		clips.low_facing.reach = Reach(clips.low_facing, 1);
		clips.wall_back.reach = Reach(clips.wall_back, -1);
		clips.low_back.reach = Reach(clips.low_back, -1);
		// Facing a wall, the soldier stops short of it instead of touching it.
		clips.wall_facing.gap = 0.15f;

		// The vaults: the right hand plants on the wall from frame 60 to 75 of the running vault, which are frames 32 to
		// 47 of the standing one.
		auto measure = [](const MotionClip& clip, int from_frame, int to_frame, int plant_from, float& heading,
						   float& plant_ahead) {
			const Vec3 from = Ground(clip.frames[from_frame].positions[0]);
			const Vec3 travel = Ground(clip.frames[to_frame].positions[0]) - from;
			heading = HeadingOf(travel);
			Vec3 plant;
			for (int f = plant_from; f < plant_from + 16; ++f)
				plant = plant + Ground(clip.frames[f].positions[kRightHand]) * (1.0f / 16.0f);
			const Vec3 ahead = plant - from;
			plant_ahead = (ahead.x * travel.x + ahead.z * travel.z) / travel.Length();
		};
		measure(clips.vault, clips.vault_from, clips.vault_to, 60, clips.vault_heading, clips.vault_plant);
		measure(clips.vault_stand, clips.vault_stand_from, clips.vault_stand_to, 32, clips.vault_stand_heading,
			clips.vault_stand_plant);
		return true;
	}

	std::vector<Leg> PaceLegs(const std::vector<Leg>& legs, const Vec3& start, Pace pace)
	{
		// The longest piece the model covers at the pace within its transition lengths.
		const float max_piece = pace == Pace::Sneak ? 1.1f : 1.6f;
		std::vector<Leg> pieces;
		Vec3 previous = start;
		for (const Leg& leg : legs)
		{
			const Vec3 travel = leg.end - previous;
			const int count = std::max(1, int(std::ceil(travel.Length() / max_piece)));
			for (int p = 1; p <= count; ++p)
			{
				Leg piece = leg;
				if (p < count)
				{
					piece.end = previous + travel * (float(p) / float(count));
					piece.heading = HeadingOf(travel);
					piece.last = false;
					piece.turn = 0;
					piece.vault = false;
				}
				// A vault after sneaking or walking starts from standing still.
				piece.standing_vault = piece.vault;
				piece.sneak = pace == Pace::Sneak && !piece.last && !piece.vault;
				piece.walk = pace == Pace::Walk && !piece.last && !piece.vault;
				piece.min_tokens = piece.vault ? 8 : 10;
				piece.max_tokens = piece.vault ? 12 : 14;
				piece.path_from = piece.path_to = -1;
				pieces.push_back(piece);
			}
			// A vault ends on the tile behind the wall, as far past the wall as the leg ends before it.
			previous = leg.vault ? leg.vault_wall * 2.0f - leg.end : leg.end;
		}
		return pieces;
	}

	GoalPose ChooseGoalPose(const Grid& grid, const ClipSet& clips, Tile goal, float heading, bool crouched)
	{
		GoalPose pose{crouched ? &clips.crouch : &clips.stand, grid.Center(goal), heading};
		float cover_heading = 0;
		const Cell cover = grid.CoverAt(goal, &cover_heading);
		if (cover == Cell::Free)
			return pose;
		const bool facing = std::abs(WrapAngle(heading - cover_heading)) <= kPi * 0.5f;
		const bool half = cover == Cell::HalfCover;
		pose.clip = half ? (facing ? &clips.low_facing : &clips.low_back) : (facing ? &clips.wall_facing : &clips.wall_back);
		pose.heading = facing ? cover_heading : WrapAngle(cover_heading + kPi);
		// The cover's face, from the goal tile's center.
		const Vec3 toward{std::round(std::sin(cover_heading)), 0, std::round(std::cos(cover_heading))};
		const Obstacle box = grid.CoverBox({goal.x + int(toward.x), goal.y + int(toward.z)});
		const Vec3 c = pose.ground;
		const float face = toward.x > 0 ? box.min.x - c.x : toward.x < 0 ? c.x - box.max.x : toward.z > 0 ? box.min.z - c.z
																										   : c.z - box.max.z;
		const float distance = face - pose.clip->reach - pose.clip->gap;
		pose.ground = pose.ground + toward * distance;
		return pose;
	}

	Contact CheckContacts(const std::vector<ModelFrame>& frames, const std::vector<Obstacle>& obstacles)
	{
		Contact contact;
		for (size_t index = 0; index < frames.size(); ++index)
		{
			const ModelFrame& frame = frames[index];
			float frame_depth = 0;
			int frame_obstacle = -1, frame_joint = -1;
			for (const auto& [joint, radius] : kBodyPoints)
			{
				const Vec3& p = frame.positions[joint];
				for (size_t o = 0; o < obstacles.size(); ++o)
				{
					const Obstacle& box = obstacles[o];
					// Penetration of a sphere into the box: how far its surface reaches past the nearest face.
					const float inside_x = std::min(p.x - box.min.x, box.max.x - p.x) + radius;
					const float inside_y = std::min(p.y - box.min.y, box.max.y - p.y) + radius;
					const float inside_z = std::min(p.z - box.min.z, box.max.z - p.z) + radius;
					const float depth = std::min({inside_x, inside_y, inside_z});
					if (depth > frame_depth)
					{
						frame_depth = depth;
						frame_obstacle = int(o);
						frame_joint = joint;
					}
				}
			}
			if (frame_depth > 0)
			{
				++contact.frames;
				if (frame_depth > contact.depth)
				{
					contact.depth = frame_depth;
					contact.hips = frame.positions[0];
					contact.obstacle = frame_obstacle;
					contact.joint = frame_joint;
					contact.frame = int(index);
				}
			}
		}
		return contact;
	}

	void MotionPlayer::SetLoop(const MotionClip* clip, int start_frame, const Placement& placement)
	{
		loop_ = clip;
		loop_frame_ = start_frame;
		loop_placement_ = placement;
		if (history_.empty())
		{
			// Start with a full context.
			for (int i = 0; i < 4; ++i)
				Push(FromLoop());
		}
	}

	void MotionPlayer::Queue(const ModelFrame& frame)
	{
		queue_.push_back(frame);
	}

	PlayedFrame MotionPlayer::FromLoop()
	{
		PlayedFrame frame;
		// The last frames blend into the first frames of the loop, which then continues after them.
		const int end = loop_->Frames();
		const int blend = std::min(kLoopBlend, (end - loop_->loop_start) / 2);
		if (loop_frame_ >= end)
			loop_frame_ = loop_->loop_start + blend;
		frame.model = ModelFrameFromClip(*loop_, loop_frame_, loop_placement_);
		if (loop_frame_ >= end - blend)
		{
			const int i = loop_frame_ - (end - blend);
			const ModelFrame start = ModelFrameFromClip(*loop_, loop_->loop_start + i, loop_placement_);
			const float t = float(i + 1) / float(blend);
			for (int k = 0; k < kModelJoints; ++k)
			{
				frame.model.positions[k] = frame.model.positions[k] + (start.positions[k] - frame.model.positions[k]) * t;
				frame.model.rotations[k] = BlendRotations(frame.model.rotations[k], start.rotations[k], t);
			}
		}
		frame.pose = PoseFromModel(skeleton_, frame.model);
		++loop_frame_;
		return frame;
	}

	void MotionPlayer::Push(const PlayedFrame& frame)
	{
		current_ = frame;
		history_.push_back(frame.model);
		while (history_.size() > 4)
			history_.pop_front();
	}

	void MotionPlayer::Advance(float seconds)
	{
		clock_ += seconds;
		while (clock_ >= kFrameTime)
		{
			clock_ -= kFrameTime;
			if (!queue_.empty())
			{
				PlayedFrame frame;
				frame.model = queue_.front();
				frame.pose = PoseFromModel(skeleton_, frame.model);
				queue_.pop_front();
				Push(frame);
			}
			else if (loop_)
			{
				Push(FromLoop());
			}
		}
	}

	std::array<ModelFrame, 4> MotionPlayer::Tail() const
	{
		std::vector<ModelFrame> all(history_.begin(), history_.end());
		all.insert(all.end(), queue_.begin(), queue_.end());
		std::array<ModelFrame, 4> tail;
		for (int i = 0; i < 4; ++i)
			tail[i] = all[all.size() - 4 + i];
		return tail;
	}

	std::array<ModelFrame, 4> MovePlanner::Target(const Leg& leg, const MotionClip& goal_clip, const MotionClip*& clip,
		Placement& placement) const
	{
		std::array<ModelFrame, 4> frames;
		if (leg.last)
		{
			// Arrive in the goal pose: the first four frames of its settled part.
			clip = &goal_clip;
			placement = PlaceClipFrame(*clip, clip->loop_start, leg.end, leg.heading);
			for (int f = 0; f < 4; ++f)
				frames[f] = ModelFrameFromClip(*clip, clip->loop_start + f, placement);
			return frames;
		}
		if (leg.vault)
		{
			// Arrive in the vault clip, running or standing, placed so that the hand plants over the wall's center.
			const bool standing = leg.standing_vault;
			clip = standing ? &clips_.vault_stand : &clips_.vault;
			const int from = standing ? clips_.vault_stand_from : clips_.vault_from;
			const Vec3 forward{std::sin(leg.heading), 0, std::cos(leg.heading)};
			placement.yaw = WrapAngle(leg.heading - (standing ? clips_.vault_stand_heading : clips_.vault_heading));
			const Vec3 hips = YawRotation(placement.yaw) * Ground(clip->frames[from].positions[0]);
			placement.offset =
				Ground(leg.vault_wall) - forward * (standing ? clips_.vault_stand_plant : clips_.vault_plant) - hips;
			for (int f = 0; f < 4; ++f)
				frames[f] = ModelFrameFromClip(*clip, from + f, placement);
			return frames;
		}
		// Pass through running, walking or sneaking: four frames of the clip made in place, then moved forward at the
		// target speed so the model sees the velocity.
		clip = leg.sneak ? &clips_.sneak : leg.walk ? &clips_.walk : &clips_.run;
		const int first = leg.sneak ? clips_.sneak_frame : leg.walk ? clips_.walk_frame : clips_.run_frame;
		// Sharp corners are taken slower; at full speed the model turns wide.
		const float slow = std::clamp(1.0f - leg.turn / 1.5708f * 0.65f, 0.35f, 1.0f);
		const float speed = (leg.sneak ? sneak_speed : leg.walk ? walk_speed : run_speed_ * slow) * leg.speed_scale;
		placement = PlaceClipFrame(*clip, first, leg.end, leg.heading);
		const Vec3 forward{std::sin(leg.heading), 0, std::cos(leg.heading)};
		for (int f = 0; f < 4; ++f)
		{
			frames[f] = ModelFrameFromClip(*clip, first + f, placement);
			const Vec3 shift = forward * (speed * kFrameTime * float(f)) -
				(Ground(frames[f].positions[0]) - Ground(frames[0].positions[0]));
			for (Vec3& p : frames[f].positions)
				p = p + shift;
		}
		return frames;
	}

	bool MovePlanner::Generate(const std::array<ModelFrame, 4>& context, const std::array<ModelFrame, 4>& target,
		std::vector<ModelFrame>& frames, LegReport& report, std::string* error, int leg_min_tokens, int leg_max_tokens)
	{
		std::vector<float> context_joints, context_rotations, target_joints, target_rotations;
		Flatten(context, context_joints, context_rotations);
		Flatten(target, target_joints, target_rotations);
		motion::TransitionRequest request;
		request.context = {context_joints.data(), context_rotations.data(), 4, kModelJoints};
		request.target = {target_joints.data(), target_rotations.data(), 4, kModelJoints};
		request.num_target_poses = 4;
		request.min_tokens = leg_min_tokens > 0 ? leg_min_tokens : min_tokens;
		request.max_tokens = leg_max_tokens > 0 ? leg_max_tokens : max_tokens;
		std::vector<motion::Transition> transitions;
		const auto start = std::chrono::steady_clock::now();
		if (!bricks_.Generate({&request, 1}, transitions, error))
			return false;
		const auto stop = std::chrono::steady_clock::now();

		const motion::Transition& t = transitions[0];
		report.tokens = t.tokens;
		report.frames = t.frames;
		report.distance = (Ground(target[0].positions[0]) - Ground(context[3].positions[0])).Length();
		report.milliseconds = std::chrono::duration<double, std::milli>(stop - start).count();
		// The transition starts on the last context frame, already played, and ends on the first target frame; the
		// other target frames follow it.
		frames.clear();
		for (int f = 1; f < t.frames; ++f)
			frames.push_back(FromTransition(t, f));
		for (int f = 1; f < 4; ++f)
			frames.push_back(target[f]);
		return true;
	}

	bool MovePlanner::Plan(MotionPlayer& player, std::vector<Leg>& legs, const std::vector<Vec3>& path,
		const MotionClip& goal_clip, const std::vector<Obstacle>& obstacles, std::vector<LegReport>& reports, std::string* error)
	{
		reports.clear();
		std::array<ModelFrame, 4> tail = player.Tail();
		std::vector<ModelFrame> planned;
		const MotionClip* end_clip = nullptr;
		Placement end_placement;
		int attempts = 0;
		float best_depth = INFINITY;
		std::vector<ModelFrame> best_frames;
		LegReport best_report;
		for (size_t i = 0; i < legs.size();)
		{
			const Leg& leg = legs[i];
			const MotionClip* clip = nullptr;
			Placement placement;
			const std::array<ModelFrame, 4> target = Target(leg, goal_clip, clip, placement);
			std::vector<ModelFrame> frames;
			LegReport report;
			report.detour = leg.detour;
			if (!Generate(tail, target, frames, report, error, leg.min_tokens, leg.max_tokens))
				return false;
			const Contact contact = CheckContacts(frames, obstacles);
			report.contact_frames = contact.frames;
			report.depth = contact.depth;

			// The body goes through an obstacle: split the leg at the path tile between its ends that is nearest the
			// contact, and generate both parts. Tile centers keep clear of every cover.
			if (contact.depth > kDetourDepth && leg.path_from >= 0 && leg.path_to - leg.path_from >= 2)
			{
				int split = leg.path_from + 1;
				float nearest = INFINITY;
				for (int k = leg.path_from + 1; k < leg.path_to; ++k)
				{
					const float d = (Ground(path[k]) - Ground(contact.hips)).Length();
					if (d < nearest)
					{
						nearest = d;
						split = k;
					}
				}
				Leg first = leg;
				first.end = path[split];
				first.heading = HeadingOf(leg.end - path[split]);
				first.last = false;
				first.detour = true;
				first.vault = false;
				first.path_to = split;
				legs[i].path_from = split;
				legs.insert(legs.begin() + i, first);
				continue;
			}
			// A leg that cannot be split is tried again arriving slower, twice at most; the cleanest attempt is kept.
			if (contact.depth > kDetourDepth && !leg.last && !leg.vault && attempts < 2)
			{
				if (contact.depth < best_depth)
				{
					best_depth = contact.depth;
					best_frames = frames;
					best_report = report;
				}
				legs[i].speed_scale *= 0.6f;
				++attempts;
				continue;
			}
			if (attempts > 0 && contact.depth > best_depth)
			{
				frames = best_frames;
				report = best_report;
			}
			attempts = 0;
			best_depth = INFINITY;

			// A vault plays on over the wall until just after landing.
			if (leg.vault)
			{
				const bool standing = leg.standing_vault;
				const int from = standing ? clips_.vault_stand_from : clips_.vault_from;
				const int to = standing ? clips_.vault_stand_to : clips_.vault_to;
				for (int f = from + 4; f < to; ++f)
					frames.push_back(ModelFrameFromClip(*clip, f, placement));
			}
			reports.push_back(report);
			planned.insert(planned.end(), frames.begin(), frames.end());
			tail = LastFour(tail, frames);
			if (leg.last)
			{
				end_clip = clip;
				end_placement = placement;
			}
			++i;
		}
		for (const ModelFrame& frame : planned)
			player.Queue(frame);
		if (end_clip)
			player.SetLoop(end_clip, end_clip->loop_start + 4, end_placement);
		return true;
	}
}
