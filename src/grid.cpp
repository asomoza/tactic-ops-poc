#include "grid.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace tactics
{
	namespace
	{
		// The extra cost of a vault, in tiles, over walking the two tiles it crosses.
		constexpr float kVaultCost = 1.0f;
	}

	Grid::Grid(int width, int height, float tile_size)
		: width_(width), height_(height), tile_(tile_size), cells_(size_t(width) * height, Cell::Free)
	{
	}

	Vec3 Grid::Center(Tile t) const
	{
		return {(float(t.x) + 0.5f) * tile_, 0.0f, (float(t.y) + 0.5f) * tile_};
	}

	bool Grid::TileAt(const Vec3& position, Tile& tile) const
	{
		tile = {int(std::floor(position.x / tile_)), int(std::floor(position.z / tile_))};
		return Inside(tile);
	}

	bool Grid::AlongX(Tile tile) const
	{
		auto half = [&](Tile t) { return Inside(t) && At(t) == Cell::HalfCover; };
		return half({tile.x - 1, tile.y}) || half({tile.x + 1, tile.y}) ||
			!(half({tile.x, tile.y - 1}) || half({tile.x, tile.y + 1}));
	}

	Obstacle Grid::CoverBox(Tile tile) const
	{
		const Vec3 c = Center(tile);
		const float length = tile_ * kCoverLength;
		float x = length, z = length, height = kFullCoverHeight;
		if (At(tile) == Cell::HalfCover)
		{
			(AlongX(tile) ? z : x) = kHalfCoverDepth * 0.5f;
			height = kHalfCoverHeight;
		}
		return {{c.x - x, 0, c.z - z}, {c.x + x, height, c.z + z}};
	}

	std::vector<Obstacle> Grid::Obstacles() const
	{
		std::vector<Obstacle> boxes;
		for (int y = 0; y < height_; ++y)
			for (int x = 0; x < width_; ++x)
				if (At({x, y}) != Cell::Free)
					boxes.push_back(CoverBox({x, y}));
		return boxes;
	}

	std::vector<Tile> Grid::FindPath(Tile start, Tile goal) const
	{
		if (!Inside(start) || !Walkable(goal))
			return {};
		const int count = width_ * height_;
		std::vector<float> cost(count, INFINITY);
		std::vector<int> from(count, -1);
		using Entry = std::pair<float, int>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
		auto index = [&](Tile t) { return t.y * width_ + t.x; };
		auto estimate = [&](Tile t) {
			const float dx = std::abs(float(t.x - goal.x)), dy = std::abs(float(t.y - goal.y));
			return std::max(dx, dy) + (std::sqrt(2.0f) - 1.0f) * std::min(dx, dy);
		};
		cost[index(start)] = 0;
		open.push({estimate(start), index(start)});
		while (!open.empty())
		{
			const int current = open.top().second;
			open.pop();
			const Tile t{current % width_, current / width_};
			if (t == goal)
				break;
			for (int dy = -1; dy <= 1; ++dy)
				for (int dx = -1; dx <= 1; ++dx)
				{
					if (dx == 0 && dy == 0)
						continue;
					const Tile n{t.x + dx, t.y + dy};
					if (!Walkable(n))
						continue;
					if (dx != 0 && dy != 0 && (!Walkable({t.x + dx, t.y}) || !Walkable({t.x, t.y + dy})))
						continue;
					const float step = (dx != 0 && dy != 0) ? std::sqrt(2.0f) : 1.0f;
					const float next = cost[current] + step;
					if (next < cost[index(n)])
					{
						cost[index(n)] = next;
						from[index(n)] = current;
						open.push({next + estimate(n), index(n)});
					}
				}
			// Vaults: straight across a low wall that crosses the direction of travel, to the free tile behind it.
			const Tile directions[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
			for (const Tile& d : directions)
			{
				const Tile wall{t.x + d.x, t.y + d.y}, behind{t.x + 2 * d.x, t.y + 2 * d.y};
				if (!Inside(wall) || At(wall) != Cell::HalfCover || AlongX(wall) != (d.x == 0) ||
					!Walkable(behind))
					continue;
				const float next = cost[current] + 2.0f + kVaultCost;
				if (next < cost[index(behind)])
				{
					cost[index(behind)] = next;
					from[index(behind)] = current;
					open.push({next + estimate(behind), index(behind)});
				}
			}
		}
		if (from[index(goal)] < 0 && goal != start)
			return {};
		std::vector<Tile> path;
		for (int i = index(goal); i >= 0; i = from[i])
		{
			const Tile t{i % width_, i / width_};
			// A vault skips its wall's tile; put it back on the path.
			if (!path.empty() && (std::abs(path.back().x - t.x) == 2 || std::abs(path.back().y - t.y) == 2))
				path.push_back({(path.back().x + t.x) / 2, (path.back().y + t.y) / 2});
			path.push_back(t);
		}
		std::reverse(path.begin(), path.end());
		return path;
	}

	Cell Grid::CoverAt(Tile tile, float* heading) const
	{
		Cell best = Cell::Free;
		const Tile sides[] = {{tile.x + 1, tile.y}, {tile.x - 1, tile.y}, {tile.x, tile.y + 1}, {tile.x, tile.y - 1}};
		for (const Tile& side : sides)
		{
			if (!Inside(side) || At(side) == Cell::Free || At(side) < best)
				continue;
			best = At(side);
			if (heading)
				*heading = HeadingOf(Center(side) - Center(tile));
		}
		return best;
	}

	bool Grid::Clear(const Vec3& from, const Vec3& to, float clearance) const
	{
		const Vec3 delta = to - from;
		const int steps = std::max(1, int(delta.Length() / 0.1f));
		const float offsets[][2] = {{0, 0}, {clearance, 0}, {-clearance, 0}, {0, clearance}, {0, -clearance}};
		for (int i = 0; i <= steps; ++i)
		{
			const Vec3 p = from + delta * (float(i) / float(steps));
			for (const auto& o : offsets)
			{
				Tile t;
				if (!TileAt({p.x + o[0], 0, p.z + o[1]}, t) || !Walkable(t))
					return false;
			}
		}
		return true;
	}

	std::vector<Leg> Grid::Legs(const std::vector<Tile>& path, float max_length) const
	{
		std::vector<Leg> legs;
		if (path.size() < 2)
			return legs;
		// Corners: from each one, the farthest later path tile reachable in a straight line with room for the body.
		// The tiles in front of and behind a vaulted wall are always corners, and the run between them is the vault.
		auto vault_at = [&](size_t i) { return i + 1 < path.size() && At(path[i + 1]) == Cell::HalfCover; };
		std::vector<size_t> corners{0};
		size_t at = 0;
		while (at + 1 < path.size())
		{
			size_t next = at + 1;
			if (vault_at(at))
				next = at + 2;
			else
			{
				size_t last = path.size() - 1;
				for (size_t i = at + 1; i < path.size(); ++i)
					if (vault_at(i))
					{
						last = i;
						break;
					}
				for (size_t candidate = last; candidate > at + 1; --candidate)
					if (Clear(Center(path[at]), Center(path[candidate]), 0.5f))
					{
						next = candidate;
						break;
					}
			}
			corners.push_back(next);
			at = next;
		}
		// Long runs become several legs of equal length; a vault marks the leg before it.
		for (size_t i = 1; i < corners.size(); ++i)
		{
			if (vault_at(corners[i - 1]))
			{
				if (legs.empty())
				{
					// A vault from the start tile: a short leg in place into the standing vault.
					Leg start;
					start.end = Center(path[corners[i - 1]]);
					start.standing_vault = true;
					start.min_tokens = 6;
					start.max_tokens = 8;
					legs.push_back(start);
				}
				legs.back().vault = true;
				legs.back().vault_wall = Center(path[corners[i - 1] + 1]);
				legs.back().heading = HeadingOf(Center(path[corners[i]]) - Center(path[corners[i - 1]]));
				if (i + 1 == corners.size())
				{
					// The goal is right behind the wall: one more leg to stop on it.
					Leg leg;
					leg.end = Center(path[corners[i]]);
					leg.heading = legs.back().heading;
					leg.last = true;
					legs.push_back(leg);
				}
				continue;
			}
			const Vec3 from = Center(path[corners[i - 1]]), to = Center(path[corners[i]]);
			const float length = (to - from).Length();
			const int parts = std::max(1, int(std::ceil(length / max_length)));
			const int span = int(corners[i] - corners[i - 1]);
			for (int k = 1; k <= parts; ++k)
			{
				Leg leg;
				leg.end = from + (to - from) * (float(k) / float(parts));
				leg.heading = HeadingOf(to - from);
				leg.last = i + 1 == corners.size() && k == parts;
				leg.path_from = int(corners[i - 1]) + span * (k - 1) / parts;
				leg.path_to = int(corners[i - 1]) + span * k / parts;
				legs.push_back(leg);
			}
		}
		// Each intermediate leg arrives facing the next leg's direction, or the vault's; the last faces its cover.
		for (size_t i = 0; i + 1 < legs.size(); ++i)
		{
			const Vec3 from = i == 0 ? Center(path.front()) : legs[i - 1].end;
			const float travel = HeadingOf(legs[i].end - from);
			if (!legs[i].vault)
				legs[i].heading = HeadingOf(legs[i + 1].end - legs[i].end);
			legs[i].turn = std::abs(WrapAngle(legs[i].heading - travel));
		}
		float cover_heading = 0;
		if (CoverAt(path.back(), &cover_heading) != Cell::Free)
			legs.back().heading = cover_heading;
		return legs;
	}

	void BuildYard(Grid& grid)
	{
		const Tile half[] = {{4, 2}, {4, 3}, {7, 6}, {8, 6}, {2, 8}, {3, 8}, {9, 9}, {6, 10}, {10, 1}};
		// (6, 3) and (7, 3) shield the soldier's start at (7, 2) from the north.
		const Tile full[] = {{6, 3}, {6, 4}, {7, 3}, {2, 5}, {10, 3}, {9, 7}, {5, 8}, {5, 7}, {1, 1}};
		for (const Tile& t : half)
			grid.Set(t, Cell::HalfCover);
		for (const Tile& t : full)
			grid.Set(t, Cell::FullCover);
	}
}
