// The SOMA character rig: the mesh skeleton, poses from clips or from MotionBricks, and skinning.
//
// Two rotation conventions meet here. The mesh skeleton and NVIDIA's runtime compose world rotations from the
// skeleton's joint frames ("runtime"). The motion module and Kimodo clips use SOMA global rotations relative to the
// neutral pose ("model"); motion::RuntimeToModelRotations converts between them.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "math3.h"

namespace tactics
{
	constexpr int kModelJoints = 30;

	struct SomaSkeleton
	{
		std::vector<std::string> names;
		std::vector<int> parents;
		std::vector<Transform> rest; // local
		std::vector<Transform> bind; // world, the pose the mesh is skinned in
		std::array<int, kModelJoints> model_joints{}; // mesh joint of each SOMA30 joint
		std::vector<int> model_of; // SOMA30 joint whose rotation drives each mesh joint, or -1 for the rest pose

		int Joint(const std::string& name) const;
	};

	bool LoadSomaSkeleton(const std::string& path, SomaSkeleton& skeleton, std::string* error);

	// One SOMA30 frame as the motion module takes and returns it: joint positions and model-convention rotations.
	struct ModelFrame
	{
		std::array<Vec3, kModelJoints> positions;
		std::array<Mat3, kModelJoints> rotations;
	};

	// A mesh pose: the runtime-convention local rotation of every joint (the root's is its world rotation) and the
	// root's world position. Bones keep the mesh's rest lengths.
	struct RigPose
	{
		std::vector<Mat3> local;
		Vec3 root;
	};

	void PoseWorld(const SomaSkeleton& skeleton, const RigPose& pose, std::vector<Transform>& world);
	RigPose PoseFromModel(const SomaSkeleton& skeleton, const ModelFrame& frame);

	// Where a clip is played: world = yaw rotation about +Y, then offset, applied to the clip's own world.
	struct Placement
	{
		float yaw = 0;
		Vec3 offset;
		Transform AsTransform() const { return {YawRotation(yaw), offset}; }
	};

	// The heading of a model frame, from its hip joints.
	float FrameHeading(const ModelFrame& frame);
}
