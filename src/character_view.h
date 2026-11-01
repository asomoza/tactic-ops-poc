// Draws the SOMA character in a Wicked scene, skinned from rig poses.
#pragma once

#include <string>
#include <vector>

#include "WickedEngine.h"
#include "rig.h"

namespace tactics
{
	class CharacterView
	{
	public:
		// Imports the glTF character into the scene and matches its bones to the skeleton by name.
		bool Load(wi::scene::Scene& scene, const std::string& glb_path, const SomaSkeleton& skeleton, std::string* error);
		// Poses the bones from the world transforms of the skeleton's joints.
		void Apply(wi::scene::Scene& scene, const std::vector<Transform>& world);

	private:
		const SomaSkeleton* skeleton_ = nullptr;
		std::vector<wi::ecs::Entity> bones_;    // per skeleton joint, or INVALID_ENTITY
		// Per skeleton joint: the bone's bind pose in Wicked's world space, including the node transforms above the
		// armature (glTF exports can carry unit scales and axis rotations there).
		std::vector<DirectX::XMFLOAT4X4> bone_bind_;
	};
}
