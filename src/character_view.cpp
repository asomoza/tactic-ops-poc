#include "character_view.h"

#include "ModelImporter.h"

using namespace DirectX;

namespace tactics
{
	namespace
	{
		// Wicked imports glTF into a left-handed space by mirroring Z: M_lh = F M_rh F with F = diag(1, 1, -1).
		Transform Mirror(const Transform& t)
		{
			Transform out = t;
			const float sign[9] = {1, 1, -1, 1, 1, -1, -1, -1, 1};
			for (int i = 0; i < 9; ++i)
				out.rotation.m[i] *= sign[i];
			out.translation.z = -out.translation.z;
			return out;
		}

		// A column-vector transform as a row-vector DirectXMath matrix.
		XMMATRIX ToMatrix(const Transform& t)
		{
			XMFLOAT4X4 m;
			for (int r = 0; r < 3; ++r)
			{
				for (int c = 0; c < 3; ++c)
					m.m[c][r] = t.rotation.m[r * 3 + c];
				m.m[r][3] = 0;
			}
			m._41 = t.translation.x;
			m._42 = t.translation.y;
			m._43 = t.translation.z;
			m._44 = 1;
			return XMLoadFloat4x4(&m);
		}
	}

	bool CharacterView::Load(wi::scene::Scene& scene, const std::string& glb_path, const SomaSkeleton& skeleton,
		std::string* error)
	{
		wi::scene::Scene imported;
		ImportModel_GLTF(glb_path, imported);
		if (imported.armatures.GetCount() == 0)
		{
			if (error)
				*error = glb_path + " has no armature";
			return false;
		}
		imported.Update(0);

		skeleton_ = &skeleton;
		bones_.assign(skeleton.names.size(), wi::ecs::INVALID_ENTITY);
		bone_bind_.assign(skeleton.names.size(), XMFLOAT4X4{});
		const wi::scene::ArmatureComponent& armature = imported.armatures[0];
		// Skinning works in the armature's space, so a bone's bind pose in world space is its inverse bind matrix
		// inverted and then moved by the armature's world matrix (which carries the export's unit scale and axes).
		XMMATRIX armature_world = XMMatrixIdentity();
		if (const auto* t = imported.transforms.GetComponent(imported.armatures.GetEntity(0)))
			armature_world = XMLoadFloat4x4(&t->world);
		for (size_t b = 0; b < armature.boneCollection.size(); ++b)
		{
			const wi::ecs::Entity bone = armature.boneCollection[b];
			const wi::scene::NameComponent* name = imported.names.GetComponent(bone);
			if (!name)
				continue;
			const int joint = skeleton.Joint(name->name);
			if (joint < 0)
				continue;
			bones_[joint] = bone;
			XMStoreFloat4x4(&bone_bind_[joint],
				XMMatrixInverse(nullptr, XMLoadFloat4x4(&armature.inverseBindMatrices[b])) * armature_world);
		}

		// A neutral material for meshes that come without one.
		const wi::ecs::Entity material = imported.Entity_CreateMaterial("character");
		wi::scene::MaterialComponent* look = imported.materials.GetComponent(material);
		look->SetBaseColor(XMFLOAT4(0.62f, 0.66f, 0.72f, 1));
		look->SetRoughness(0.6f);
		for (size_t i = 0; i < imported.meshes.GetCount(); ++i)
			for (auto& subset : imported.meshes[i].subsets)
				if (subset.materialID == wi::ecs::INVALID_ENTITY || !imported.materials.Contains(subset.materialID))
					subset.materialID = material;

		// Bones are posed directly in world space.
		for (wi::ecs::Entity bone : bones_)
			if (bone != wi::ecs::INVALID_ENTITY)
				imported.Component_Detach(bone);
		scene.Merge(imported);
		return true;
	}

	void CharacterView::Apply(wi::scene::Scene& scene, const std::vector<Transform>& world)
	{
		for (size_t j = 0; j < bones_.size(); ++j)
		{
			wi::scene::TransformComponent* transform = scene.transforms.GetComponent(bones_[j]);
			if (!transform)
				continue;
			// The skinning change maps the skeleton's bind pose to its posed transform in world space; the bone gets the
			// same change applied after its own bind world matrix.
			const Transform skin = Mirror(world[j] * skeleton_->bind[j].Inverse());
			const XMMATRIX bone = XMLoadFloat4x4(&bone_bind_[j]) * ToMatrix(skin);
			transform->ClearTransform();
			transform->MatrixTransform(bone);
			transform->SetDirty();
		}
	}
}
