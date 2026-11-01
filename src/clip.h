// Motion clips from Kinetic Artisan (Kimodo motion NPZ, SOMA77 at 30 FPS), kept as SOMA30 model frames: Kimodo's
// global rotations are already in the model's convention.
#pragma once

#include <string>
#include <vector>

#include "rig.h"

namespace tactics
{
	struct MotionClip
	{
		std::string name;
		std::vector<ModelFrame> frames;
		// The first frame of the settled part: idles loop back to it and moves arrive on it.
		int loop_start = 0;
		// For cover poses: how far the body reaches toward the cover from the hips' ground point, in meters, over the
		// settled part.
		float reach = 0;
		// For cover poses: the space kept between the body and the cover's face, in meters.
		float gap = 0;

		int Frames() const { return int(frames.size()); }
	};

	bool LoadMotionClip(const std::string& path, MotionClip& clip, std::string* error);

	// The clip reflected left to right across its own X = 0 plane: a left turn becomes a right turn.
	MotionClip MirrorClip(const MotionClip& clip);

	ModelFrame ModelFrameFromClip(const MotionClip& clip, int frame, const Placement& placement);
	RigPose PoseFromClip(const SomaSkeleton& skeleton, const MotionClip& clip, int frame, const Placement& placement);
	// The placement that puts a clip frame's hips over ground point `ground` (y ignored), facing `heading`.
	Placement PlaceClipFrame(const MotionClip& clip, int frame, const Vec3& ground, float heading);
}
