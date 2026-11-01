// Small vector and rotation helpers. Matrices are row-major 3x3 acting on column vectors.
#pragma once

#include <array>
#include <cmath>

namespace tactics
{
	struct Vec3
	{
		float x = 0, y = 0, z = 0;
		Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
		Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
		Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
		float Length() const { return std::sqrt(x * x + y * y + z * z); }
	};

	struct Mat3
	{
		std::array<float, 9> m = {1, 0, 0, 0, 1, 0, 0, 0, 1};

		Mat3 operator*(const Mat3& b) const
		{
			Mat3 c;
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					c.m[i * 3 + j] = m[i * 3] * b.m[j] + m[i * 3 + 1] * b.m[3 + j] + m[i * 3 + 2] * b.m[6 + j];
			return c;
		}
		Vec3 operator*(const Vec3& v) const
		{
			return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
				m[6] * v.x + m[7] * v.y + m[8] * v.z};
		}
		Mat3 Transposed() const { return {{m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]}}; }
	};

	// A rigid transform: p' = rotation * p + translation.
	struct Transform
	{
		Mat3 rotation;
		Vec3 translation;
		Transform operator*(const Transform& b) const { return {rotation * b.rotation, rotation * b.translation + translation}; }
		Transform Inverse() const
		{
			const Mat3 r = rotation.Transposed();
			return {r, (r * translation) * -1.0f};
		}
	};

	inline Mat3 AxisRotation(int axis, float radians)
	{
		const float c = std::cos(radians), s = std::sin(radians);
		if (axis == 0)
			return {{1, 0, 0, 0, c, -s, 0, s, c}};
		if (axis == 1)
			return {{c, 0, s, 0, 1, 0, -s, 0, c}};
		return {{c, -s, 0, s, c, 0, 0, 0, 1}};
	}

	inline Mat3 YawRotation(float radians)
	{
		return AxisRotation(1, radians);
	}

	// Heading about +Y of the forward direction (sin h, 0, cos h), from the right and left hip joints, as the model
	// measures it.
	inline float HeadingFromLegs(const Vec3& right, const Vec3& left)
	{
		const float ax = right.x - left.x, az = right.z - left.z;
		return std::hypot(ax, az) > 1e-6f ? std::atan2(az, -ax) : 0.0f;
	}

	// A rotation quaternion, for blending rotations.
	struct Quat
	{
		float w = 1, x = 0, y = 0, z = 0;
	};

	inline Quat QuatFromMat3(const Mat3& r)
	{
		const auto& m = r.m;
		Quat q;
		const float trace = m[0] + m[4] + m[8];
		if (trace > 0)
		{
			const float s = std::sqrt(trace + 1.0f) * 2;
			q = {0.25f * s, (m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s};
		}
		else if (m[0] > m[4] && m[0] > m[8])
		{
			const float s = std::sqrt(1.0f + m[0] - m[4] - m[8]) * 2;
			q = {(m[7] - m[5]) / s, 0.25f * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s};
		}
		else if (m[4] > m[8])
		{
			const float s = std::sqrt(1.0f + m[4] - m[0] - m[8]) * 2;
			q = {(m[2] - m[6]) / s, (m[1] + m[3]) / s, 0.25f * s, (m[5] + m[7]) / s};
		}
		else
		{
			const float s = std::sqrt(1.0f + m[8] - m[0] - m[4]) * 2;
			q = {(m[3] - m[1]) / s, (m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25f * s};
		}
		return q;
	}

	inline Mat3 Mat3FromQuat(const Quat& q)
	{
		const float w = q.w, x = q.x, y = q.y, z = q.z;
		return {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z),
			1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
	}

	// Normalized linear blend from a to b, along the shorter way round.
	inline Mat3 BlendRotations(const Mat3& a, const Mat3& b, float t)
	{
		const Quat p = QuatFromMat3(a);
		Quat q = QuatFromMat3(b);
		if (p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z < 0)
			q = {-q.w, -q.x, -q.y, -q.z};
		Quat r{p.w + (q.w - p.w) * t, p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t, p.z + (q.z - p.z) * t};
		const float n = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
		return Mat3FromQuat({r.w / n, r.x / n, r.y / n, r.z / n});
	}

	inline float WrapAngle(float radians)
	{
		return std::atan2(std::sin(radians), std::cos(radians));
	}
}
