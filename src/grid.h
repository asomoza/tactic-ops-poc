// The battlefield: square tiles, some taken by half or full cover, with 8-way paths that do not cut corners.
#pragma once

#include <cstdint>
#include <vector>

#include "math3.h"

namespace tactics
{
	enum class Cell : uint8_t
	{
		Free,
		HalfCover,
		FullCover,
	};

	// Cover stands centered on its tile. A wall (full cover) is a block; half cover is a low wall that can be vaulted,
	// running along one axis. Lengths are half the tile's width times these fractions; depth and heights in meters.
	constexpr float kCoverLength = 0.48f;
	constexpr float kHalfCoverDepth = 0.3f;
	constexpr float kHalfCoverHeight = 0.9f;
	constexpr float kFullCoverHeight = 2.2f;

	// An axis-aligned box in game coordinates that the body must not enter.
	struct Obstacle
	{
		Vec3 min, max;
	};

	struct Tile
	{
		int x = 0, y = 0;
		bool operator==(const Tile& o) const { return x == o.x && y == o.y; }
		bool operator!=(const Tile& o) const { return !(*this == o); }
	};

	// One leg of a move: run from the end of the previous leg to `end`, then face `heading`.
	struct Leg
	{
		Vec3 end;
		float heading = 0;
		bool last = false;
		bool detour = false; // added to steer the motion around an obstacle
		bool walk = false;   // pass through walking instead of running
		bool sneak = false;  // pass through sneaking in a crouch instead of running
		// The transition lengths the model may choose for this leg, in tokens; 0 keeps the planner's.
		int min_tokens = 0, max_tokens = 0;
		// The tiles of the path this leg runs between, as indices into the path; a leg that touches cover is split at
		// a path tile in between.
		int path_from = -1, path_to = -1;
		// How sharply the motion turns at the end of this leg toward the next one, in radians.
		float turn = 0;
		// Scales the arrival speed of a running or walking leg; lowered when a leg touching cover is retried.
		float speed_scale = 1;
		// The leg arrives at a low wall and vaults it: the motion follows the vault clip over the wall, whose center
		// on the ground is vault_wall, and lands heading along `heading`.
		bool vault = false;
		Vec3 vault_wall;
		// The vault starts from standing still (from the start tile, or after sneaking) instead of from a run.
		bool standing_vault = false;
	};

	class Grid
	{
	public:
		Grid(int width, int height, float tile_size);

		int Width() const { return width_; }
		int Height() const { return height_; }
		float TileSize() const { return tile_; }
		bool Inside(Tile t) const { return t.x >= 0 && t.y >= 0 && t.x < width_ && t.y < height_; }
		Cell At(Tile t) const { return cells_[t.y * width_ + t.x]; }
		void Set(Tile t, Cell cell) { cells_[t.y * width_ + t.x] = cell; }
		bool Walkable(Tile t) const { return Inside(t) && At(t) == Cell::Free; }

		// World position of a tile's center on the ground; x maps to +X and y to +Z.
		Vec3 Center(Tile t) const;
		bool TileAt(const Vec3& position, Tile& tile) const;

		// Shortest 8-way path from start to goal, both included; empty if there is none. Diagonal steps need both
		// side tiles free. A path may vault a low wall straight across, from the tile in front of it to the tile behind
		// it, but not from the start tile; the wall's tile is then on the path.
		std::vector<Tile> FindPath(Tile start, Tile goal) const;
		// The strongest cover next to a tile (4-neighbourhood) and the heading towards it; Cell::Free if none.
		Cell CoverAt(Tile tile, float* heading) const;
		// Whether the low wall on a half-cover tile runs along x (it then crosses moves along z). Low walls line up
		// with neighbouring half cover; a lone one runs along x.
		bool AlongX(Tile tile) const;
		// The box of the cover on a tile.
		Obstacle CoverBox(Tile tile) const;
		// The boxes of all the cover on the map.
		std::vector<Obstacle> Obstacles() const;
		// Whether a straight run between two ground points keeps clearance meters away from every blocked tile.
		bool Clear(const Vec3& from, const Vec3& to, float clearance) const;
		// Split a path into legs: straight runs between the corners that cannot be cut, each at most max_length
		// meters. Intermediate legs arrive facing the next leg; the last faces the cover next to the goal, or keeps
		// the direction of travel. A leg arriving in front of a low wall on the path vaults it, and the next leg
		// starts behind the wall.
		std::vector<Leg> Legs(const std::vector<Tile>& path, float max_length) const;

	private:
		int width_, height_;
		float tile_;
		std::vector<Cell> cells_;
	};

	// The game's yard, 12 by 12 tiles of 1.5 m: low walls (half cover) and walls (full cover). The soldier starts in
	// the south (low y), the patrols in the north.
	void BuildYard(Grid& grid);

	// The heading of a ground direction: forward (sin h, 0, cos h).
	inline float HeadingOf(const Vec3& direction)
	{
		return std::atan2(direction.x, direction.z);
	}
}
