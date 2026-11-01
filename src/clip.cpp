#include "clip.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>

#include "Utility/lodepng.h"
#include "motion/motionbricks.h"

namespace tactics
{
	namespace
	{
		using Bytes = std::vector<unsigned char>;

		uint64_t Read(const Bytes& b, size_t at, int size)
		{
			uint64_t v = 0;
			for (int i = size - 1; i >= 0; --i)
				v = (v << 8) | b[at + i];
			return v;
		}

		// The members of a zip archive, decompressed. Handles the zip64 records that NumPy writes.
		bool ReadZip(const Bytes& zip, std::map<std::string, Bytes>& members, std::string& error)
		{
			auto fail = [&](const std::string& what) {
				error = what;
				return false;
			};
			// The end-of-directory record is the last thing in the file, before an optional comment.
			size_t end = std::string::npos;
			for (size_t i = zip.size() >= 22 ? zip.size() - 21 : 0; i-- > 0;)
				if (Read(zip, i, 4) == 0x06054b50)
				{
					end = i;
					break;
				}
			if (end == std::string::npos)
				return fail("not a zip archive");
			uint64_t count = Read(zip, end + 10, 2), offset = Read(zip, end + 16, 4);
			if ((count == 0xffff || offset == 0xffffffff) && end >= 20 && Read(zip, end - 20, 4) == 0x07064b50)
			{
				const size_t end64 = Read(zip, end - 12, 8);
				count = Read(zip, end64 + 32, 8);
				offset = Read(zip, end64 + 48, 8);
			}
			size_t at = offset;
			for (uint64_t m = 0; m < count; ++m)
			{
				if (at + 46 > zip.size() || Read(zip, at, 4) != 0x02014b50)
					return fail("bad zip directory");
				const int method = int(Read(zip, at + 10, 2));
				uint64_t compressed = Read(zip, at + 20, 4), size = Read(zip, at + 24, 4);
				const size_t name_length = Read(zip, at + 28, 2), extra_length = Read(zip, at + 30, 2);
				const size_t comment_length = Read(zip, at + 32, 2);
				uint64_t local = Read(zip, at + 42, 4);
				const std::string name(zip.begin() + at + 46, zip.begin() + at + 46 + name_length);
				// Sizes and offsets that do not fit 32 bits are in the zip64 extra field, in this order.
				for (size_t e = at + 46 + name_length; e + 4 <= at + 46 + name_length + extra_length;)
				{
					const int id = int(Read(zip, e, 2)), length = int(Read(zip, e + 2, 2));
					if (id == 1)
					{
						size_t field = e + 4;
						if (size == 0xffffffff)
							size = Read(zip, field, 8), field += 8;
						if (compressed == 0xffffffff)
							compressed = Read(zip, field, 8), field += 8;
						if (local == 0xffffffff)
							local = Read(zip, field, 8);
					}
					e += 4 + length;
				}
				at += 46 + name_length + extra_length + comment_length;

				if (local + 30 > zip.size() || Read(zip, local, 4) != 0x04034b50)
					return fail("bad zip member " + name);
				const size_t data = local + 30 + Read(zip, local + 26, 2) + Read(zip, local + 28, 2);
				if (data + compressed > zip.size())
					return fail("truncated zip member " + name);
				Bytes& out = members[name];
				if (method == 0)
					out.assign(zip.begin() + data, zip.begin() + data + compressed);
				else if (method == 8)
				{
					unsigned char* inflated = nullptr;
					size_t inflated_size = 0;
					const unsigned status = lodepng_inflate(&inflated, &inflated_size, zip.data() + data, compressed,
						&lodepng_default_decompress_settings);
					if (status)
					{
						std::free(inflated);
						return fail("cannot inflate " + name);
					}
					out.assign(inflated, inflated + inflated_size);
					std::free(inflated);
				}
				else
					return fail("unsupported zip compression in " + name);
				if (out.size() != size)
					return fail("bad size of " + name);
			}
			return true;
		}

		// A NumPy array: its dtype descriptor, shape and raw little-endian data.
		struct Array
		{
			std::string descr;
			std::vector<size_t> shape;
			const unsigned char* data = nullptr;
			size_t bytes = 0;

			size_t Count() const
			{
				size_t n = 1;
				for (size_t s : shape)
					n *= s;
				return n;
			}
		};

		bool ParseNpy(const Bytes& npy, Array& array, std::string& error)
		{
			auto fail = [&](const std::string& what) {
				error = what;
				return false;
			};
			if (npy.size() < 10 || std::memcmp(npy.data(), "\x93NUMPY", 6) != 0)
				return fail("not a NumPy array");
			const bool long_header = npy[6] >= 2;
			const size_t header_length = long_header ? Read(npy, 8, 4) : Read(npy, 8, 2);
			const size_t start = long_header ? 12 : 10;
			if (start + header_length > npy.size())
				return fail("truncated NumPy array");
			const std::string header(npy.begin() + start, npy.begin() + start + header_length);
			const size_t descr = header.find("'descr':");
			const size_t open = header.find('\'', descr + 8);
			array.descr = header.substr(open + 1, header.find('\'', open + 1) - open - 1);
			if (header.find("'fortran_order': False") == std::string::npos)
				return fail("Fortran-ordered arrays are not supported");
			const size_t shape = header.find('(', header.find("'shape':"));
			const char* p = header.c_str() + shape + 1;
			while (*p && *p != ')')
			{
				char* next = nullptr;
				const unsigned long long v = std::strtoull(p, &next, 10);
				if (next == p)
				{
					++p;
					continue;
				}
				array.shape.push_back(size_t(v));
				p = next;
			}
			array.data = npy.data() + start + header_length;
			array.bytes = npy.size() - start - header_length;
			return true;
		}

		// A JSON string array value, such as "joint_names": [...], from the metadata.
		std::vector<std::string> JsonStrings(const std::string& json, const std::string& key)
		{
			std::vector<std::string> values;
			size_t at = json.find("\"" + key + "\"");
			if (at == std::string::npos)
				return values;
			at = json.find('[', at);
			const size_t close = json.find(']', at);
			while (at < close)
			{
				const size_t open = json.find('"', at);
				if (open >= close)
					break;
				const size_t end = json.find('"', open + 1);
				values.push_back(json.substr(open + 1, end - open - 1));
				at = end + 1;
			}
			return values;
		}

		std::string JsonValue(const std::string& json, const std::string& key)
		{
			size_t at = json.find("\"" + key + "\":");
			if (at == std::string::npos)
				return {};
			at += key.size() + 3;
			const size_t end = json.find_first_of(",}", at);
			return json.substr(at, end - at);
		}
	}

	bool LoadMotionClip(const std::string& path, MotionClip& clip, std::string* error)
	{
		std::string message;
		auto fail = [&](const std::string& what) {
			if (error)
				*error = path + ": " + what;
			return false;
		};
		std::ifstream file(path, std::ios::binary);
		if (!file)
			return fail("cannot open");
		const Bytes zip((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
		std::map<std::string, Bytes> members;
		if (!ReadZip(zip, members, message))
			return fail(message);

		Array metadata, joints, rotations;
		for (auto [name, array] : {std::pair{"kinetic_metadata.npy", &metadata}, std::pair{"posed_joints.npy", &joints},
				 std::pair{"global_rot_mats.npy", &rotations}})
		{
			const auto member = members.find(name);
			if (member == members.end())
				return fail(std::string("missing ") + name);
			if (!ParseNpy(member->second, *array, message))
				return fail(message);
		}
		// The metadata is one UTF-32 string; its JSON is ASCII.
		if (metadata.descr.rfind("<U", 0) != 0)
			return fail("unexpected metadata type " + metadata.descr);
		std::string json;
		for (size_t i = 0; i + 3 < metadata.bytes; i += 4)
			json.push_back(char(metadata.data[i]));
		if (const std::string fps = JsonValue(json, "fps"); std::strtod(fps.c_str(), nullptr) != 30.0)
			return fail("expected 30 FPS, found " + fps);
		const std::vector<std::string> names = JsonStrings(json, "joint_names");

		const size_t joint_count = names.size();
		if (joints.descr != "<f4" || rotations.descr != "<f4" || joints.shape.size() != 3 || rotations.shape.size() != 4 ||
			joints.shape[1] != joint_count || joints.shape[2] != 3 || rotations.shape[0] != joints.shape[0] ||
			rotations.shape[1] != joint_count || rotations.shape[2] != 3 || rotations.shape[3] != 3 ||
			joints.bytes < joints.Count() * 4 || rotations.bytes < rotations.Count() * 4)
			return fail("unexpected joint or rotation arrays");

		std::array<size_t, kModelJoints> source{};
		const char* const* model_names = motion::Soma30JointNames();
		for (int k = 0; k < kModelJoints; ++k)
		{
			size_t j = 0;
			while (j < joint_count && names[j] != model_names[k])
				++j;
			if (j == joint_count)
				return fail(std::string("lacks joint ") + model_names[k]);
			source[k] = j;
		}

		const size_t frames = joints.shape[0];
		clip.name = path.substr(path.find_last_of('/') + 1);
		clip.frames.resize(frames);
		for (size_t f = 0; f < frames; ++f)
			for (int k = 0; k < kModelJoints; ++k)
			{
				float p[3];
				std::memcpy(p, joints.data + ((f * joint_count + source[k]) * 3) * 4, sizeof(p));
				clip.frames[f].positions[k] = {p[0], p[1], p[2]};
				std::memcpy(clip.frames[f].rotations[k].m.data(), rotations.data + ((f * joint_count + source[k]) * 9) * 4,
					9 * 4);
			}
		return true;
	}

	MotionClip MirrorClip(const MotionClip& clip)
	{
		// Each joint takes its counterpart's motion: positions with X negated, rotations conjugated by the reflection
		// (Kimodo's rotations are relative to a left-right symmetric neutral pose).
		const char* const* names = motion::Soma30JointNames();
		std::array<int, kModelJoints> counterpart;
		for (int k = 0; k < kModelJoints; ++k)
		{
			std::string name = names[k];
			if (name.rfind("Left", 0) == 0)
				name = "Right" + name.substr(4);
			else if (name.rfind("Right", 0) == 0)
				name = "Left" + name.substr(5);
			counterpart[k] = k;
			for (int j = 0; j < kModelJoints; ++j)
				if (name == names[j])
					counterpart[k] = j;
		}
		MotionClip out = clip;
		out.name = clip.name + "_mirrored";
		for (size_t f = 0; f < clip.frames.size(); ++f)
			for (int k = 0; k < kModelJoints; ++k)
			{
				const int from = counterpart[k];
				const Vec3 p = clip.frames[f].positions[from];
				out.frames[f].positions[k] = {-p.x, p.y, p.z};
				Mat3 r = clip.frames[f].rotations[from];
				for (int e : {1, 2, 3, 6})
					r.m[e] = -r.m[e];
				out.frames[f].rotations[k] = r;
			}
		return out;
	}

	ModelFrame ModelFrameFromClip(const MotionClip& clip, int frame, const Placement& placement)
	{
		const Mat3 yaw = YawRotation(placement.yaw);
		const ModelFrame& in = clip.frames[frame];
		ModelFrame out;
		for (int k = 0; k < kModelJoints; ++k)
		{
			out.positions[k] = yaw * in.positions[k] + placement.offset;
			out.rotations[k] = yaw * in.rotations[k];
		}
		return out;
	}

	RigPose PoseFromClip(const SomaSkeleton& skeleton, const MotionClip& clip, int frame, const Placement& placement)
	{
		return PoseFromModel(skeleton, ModelFrameFromClip(clip, frame, placement));
	}

	Placement PlaceClipFrame(const MotionClip& clip, int frame, const Vec3& ground, float heading)
	{
		const ModelFrame& f = clip.frames[frame];
		Placement placement;
		placement.yaw = WrapAngle(heading - FrameHeading(f));
		const Vec3 hips = f.positions[0];
		const Vec3 turned = YawRotation(placement.yaw) * Vec3{hips.x, 0, hips.z};
		placement.offset = {ground.x - turned.x, 0, ground.z - turned.z};
		return placement;
	}
}
