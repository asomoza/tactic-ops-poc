// The proof-of-concept tactics view: one soldier on a tiled map with half and full cover, and an enemy on patrol.
// Click a tile to move; each leg of the path is a MotionBricks transition generated on the GPU, continuing from the
// previous one. If the enemy keeps the soldier in view long enough, he is detected and the round starts again.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "WickedEngine.h"
#include "character_view.h"
#include "grid.h"
#include "motion_player.h"
#include "patrol.h"
#include "rig.h"

namespace tactics
{
	struct GameSettings
	{
		std::string model_dir;   // MotionBricks diffusers weights
		std::string assets_dir;  // the tactics assets (motions)
		std::string soma_dir;    // the SOMA skeleton
		std::string motion_shaders; // the motion module's shader sources; empty for its built-in default
		std::string character;   // the soldier's glb, skinned to the SOMA skeleton
		std::string enemy;       // the enemy's glb, skinned to the SOMA skeleton
		Vision vision;           // the enemy's
		float run_speed = 3.0f;  // meters per second, for the targets of intermediate legs
		float max_leg_length = 5.0f; // meters
		float walk_distance = 3.5f;  // moves up to this long, in meters, are walked
	};

	class TacticsView : public wi::RenderPath3D
	{
	public:
		explicit TacticsView(const GameSettings& settings);
		void Load() override;
		void Update(float dt) override;
		void Compose(wi::graphics::CommandList cmd) const override;

		std::string error;
		bool Ready() const { return ready_; }
		bool Moving() const { return ready_ && player_->Busy(); }
		// Move as if the tile were clicked, with the back to the cover if asked, and center the camera on the goal; for
		// scripted captures.
		void MoveTo(Tile goal, bool back_to_cover);

	private:
		void BuildMap();
		void UpdateCamera(float dt);
		// Slide the camera's look-at point after the soldier once a move takes him toward the screen's edge, or to him
		// after Space; the camera's angle, tilt and zoom never change.
		void FollowSoldier(float dt);
		// The ground point under the cursor, in game coordinates.
		bool PointerGround(Vec3& point) const;
		// Move to a tile; with has_facing, end facing heading instead of the cover or the direction of travel.
		void Move(Tile goal, bool has_facing, float heading);
		// Crouch or stand up: in place on open ground; next to cover, for the next move.
		void ToggleCrouch();
		// Fill the detection meter while the enemy sees the soldier, drain it otherwise; full means detected.
		void UpdateDetection(float dt);
		// Everyone back to where they started: at the beginning, and after a detection.
		void Restart();
		void UpdateInput();
		void DrawOverlay(const Tile* hovered) const;

		GameSettings settings_;
		Grid grid_;
		SomaSkeleton skeleton_;
		ClipSet clips_;
		std::unique_ptr<motion::MotionBricks> bricks_;
		std::unique_ptr<MotionPlayer> player_;
		std::unique_ptr<MovePlanner> planner_;
		CharacterView character_;
		std::vector<Transform> world_;

		// Two enemies, each patrolling one side of the north.
		std::vector<std::unique_ptr<Patrol>> patrols_;
		std::vector<CharacterView> enemy_views_;
		std::vector<char> seeing_; // per patrol: sees the soldier now
		// Per patrol: where his view starts and where it points, eased toward his eyes to filter head jitter. Detection
		// and the drawn view both use these.
		std::vector<Vec3> view_eyes_;
		std::vector<float> view_heading_;
		std::vector<Transform> enemy_world_;
		float detection_ = 0;   // 0 to 1
		bool seen_ = false;     // an enemy sees the soldier now
		bool detected_ = false; // paused until the player clicks
		float elapsed_ = 0;     // seconds since the round started, stopped when detected
		bool debug_ = false;    // F1: the help, the move reports and the planned legs

		// The soldier starts in the south with his back to a wall.
		const Tile start_tile_{7, 2};
		Tile unit_tile_{7, 2};
		float unit_heading_ = 0;
		bool crouched_ = false; // moves that end on open ground end crouched
		std::vector<Obstacle> obstacles_;
		// A left press on a tile picks the goal; dragging away from it picks the final facing until release.
		bool choosing_ = false;
		Tile choice_tile_;
		bool choice_has_facing_ = false;
		float choice_heading_ = 0;
		std::vector<Tile> path_;
		std::vector<Leg> legs_;
		std::vector<LegReport> reports_;
		std::string status_;
		bool ready_ = false;
		bool has_hover_ = false; // the cursor is over a tile
		Tile hover_tile_;

		// Orbit camera around a point on the ground.
		float camera_yaw_ = 3.14159265f, camera_pitch_ = 0.95f, camera_distance_ = 11.0f;
		DirectX::XMFLOAT3 camera_focus_ = {6, 0, 6};
		DirectX::XMFLOAT2 last_pointer_ = {0, 0};
		bool follow_ = false;    // the current move may pan the camera; cleared by any camera input
		bool chasing_ = false;   // following the soldier for the rest of the move
		bool centering_ = false; // panning until the soldier is in the middle (Space)
	};
}
