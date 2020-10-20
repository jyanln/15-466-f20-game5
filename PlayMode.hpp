#include "Mode.hpp"

#include "Scene.hpp"
#include "WalkMesh.hpp"

#include <glm/glm.hpp>

#include <vector>
#include <deque>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	//functions called by main loop:
	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;
  virtual void init();
  virtual void finish_lap();

  static constexpr float max_speed = 6.0f;
  static constexpr float accel_power = 3.0f;
  static constexpr float boost_max_speed = 12.0f;
  static constexpr float boost_accel_power = 6.0f;
  static constexpr float brake_power = 3.0f;

  // How much speed decays each tick
  static constexpr float speed_decay = 0.99f;
  // How much speed decays each tick when above normal speed, applied on
  // top of normal decay
  static constexpr float boost_speed_decay = 0.98f;
  static constexpr float speed_snap = 0.3f;
  // Amount to decay on collision
  static constexpr float collision_decay = 0.5f;

	//----- game state -----

	//input tracking:
	struct Button {
		uint8_t downs = 0;
		bool pressed = 0;
	} accel, boost, brake;

	//local copy of the game scene (so code can change it during gameplay):
	Scene scene;

	//player info:
	struct Player {
		WalkPoint at;
		//transform is at player's feet and will be yawed by mouse left/right motion:
		Scene::Transform *transform = nullptr;
		//camera is at player's head and will be pitched by mouse up/down motion:
		Scene::Camera *camera = nullptr;

    float speed;
	} player;

  float current_time;
  float best_time;
};
