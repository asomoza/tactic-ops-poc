#include "rig.h"

#include <fstream>

#include "motion/motionbricks.h"

namespace tactics
{
	namespace
	{
		constexpr int kRightLeg = 26;
		constexpr int kLeftLeg = 22;

		// Through a double: the file has values below float's range, which MSVC's streams reject when reading a float.
		float ReadFloat(std::istream& in)
		{
			double v = 0;
			in >> v;
			return float(v);
		}

		Transform ReadTransform(std::istream& in)
		{
			Transform t;
			t.translation.x = ReadFloat(in);
			t.translation.y = ReadFloat(in);
			t.translation.z = ReadFloat(in);
			for (float& v : t.rotation.m)
				v = ReadFloat(in);
			return t;
		}
	}

	int SomaSkeleton::Joint(const std::string& name) const
	{
		for (size_t i = 0; i < names.size(); ++i)
			if (names[i] == name)
				return int(i);
		return -1;
	}

	bool LoadSomaSkeleton(const std::string& path, SomaSkeleton& skeleton, std::string* error)
	{
		std::ifstream in(path);
		int count = 0;
		if (!(in >> count) || count <= 0)
		{
			if (error)
				*error = "cannot read skeleton " + path;
			return false;
		}
		skeleton = {};
		for (int i = 0; i < count; ++i)
		{
			std::string name;
			int parent = -1;
			in >> name >> parent;
			skeleton.names.push_back(name);
			skeleton.parents.push_back(parent);
			skeleton.rest.push_back(ReadTransform(in));
			skeleton.bind.push_back(ReadTransform(in));
		}
		if (!in || skeleton.parents[0] != -1)
		{
			if (error)
				*error = "bad skeleton " + path;
			return false;
		}
		// A mesh joint follows a SOMA30 joint when its parent does too, as in NVIDIA's runtime; the others keep
		// their rest rotation.
		const char* const* model_names = motion::Soma30JointNames();
		skeleton.model_of.assign(count, -1);
		for (int k = 0; k < kModelJoints; ++k)
		{
			skeleton.model_joints[k] = skeleton.Joint(model_names[k]);
			if (skeleton.model_joints[k] < 0)
			{
				if (error)
					*error = std::string("skeleton lacks joint ") + model_names[k];
				return false;
			}
		}
		for (int j = 0; j < count; ++j)
			for (int k = 0; k < kModelJoints; ++k)
				if (skeleton.model_joints[k] == j)
				{
					const int parent = skeleton.parents[j];
					if (parent < 0 || skeleton.model_of[parent] >= 0)
						skeleton.model_of[j] = k;
				}
		return true;
	}

	void PoseWorld(const SomaSkeleton& skeleton, const RigPose& pose, std::vector<Transform>& world)
	{
		world.resize(skeleton.names.size());
		for (size_t j = 0; j < skeleton.names.size(); ++j)
		{
			if (skeleton.parents[j] < 0)
				world[j] = {pose.local[j], pose.root};
			else
				world[j] = world[skeleton.parents[j]] * Transform{pose.local[j], skeleton.rest[j].translation};
		}
	}

	RigPose PoseFromModel(const SomaSkeleton& skeleton, const ModelFrame& frame)
	{
		std::array<float, kModelJoints * 9> rotations;
		for (int k = 0; k < kModelJoints; ++k)
			std::copy(frame.rotations[k].m.begin(), frame.rotations[k].m.end(), rotations.begin() + k * 9);
		motion::ModelToRuntimeRotations(rotations.data(), 1);
		std::array<Mat3, kModelJoints> runtime;
		for (int k = 0; k < kModelJoints; ++k)
			std::copy(rotations.begin() + k * 9, rotations.begin() + k * 9 + 9, runtime[k].m.begin());

		RigPose pose;
		pose.root = frame.positions[0];
		pose.local.resize(skeleton.names.size());
		for (size_t j = 0; j < skeleton.names.size(); ++j)
		{
			const int k = skeleton.model_of[j];
			if (k < 0)
				pose.local[j] = skeleton.rest[j].rotation;
			else if (skeleton.parents[j] < 0)
				pose.local[j] = runtime[k];
			else
				pose.local[j] = runtime[skeleton.model_of[skeleton.parents[j]]].Transposed() * runtime[k];
		}
		return pose;
	}

	float FrameHeading(const ModelFrame& frame)
	{
		return HeadingFromLegs(frame.positions[kRightLeg], frame.positions[kLeftLeg]);
	}
}
