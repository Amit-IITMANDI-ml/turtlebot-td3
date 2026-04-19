#include <ignition/math.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <cstdlib>
#include <cmath>

#define OBSTACLE_SPEED 0.4
#define BOUNDARY 1.7  // FIX 1: Reduced from 2.7 to stay inside the maze walls
#define PI 3.14159265

namespace gazebo
{
class Obstacles : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->model = _parent;
    this->world = _parent->GetWorld();

    std::string name = _parent->GetName();
    int seed = 0;
    for (char c : name) seed += (int)c;
    std::srand(seed);

    // FIX 2: Teleport safely inside the maze at startup so they don't start outside
    double start_x = (std::rand() % 200 - 100) / 100.0; 
    double start_y = (std::rand() % 200 - 100) / 100.0;
    this->model->SetWorldPose(ignition::math::Pose3d(start_x, start_y, 0.0, 0.0, 0.0, 0.0));

    this->PickNewDirection();

    this->directionChangeInterval = 3.0 + (std::rand() % 40) / 10.0;
    this->timeSinceDirectionChange = 0.0;

    this->lastUpdateTime = this->world->SimTime();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&Obstacles::OnUpdate, this));
  }

  void OnUpdate()
  {
    gazebo::common::Time current_time = this->world->SimTime();
    double dt = (current_time - this->lastUpdateTime).Double();

    if (dt < 0.0) {
      this->lastUpdateTime = current_time;
      return;
    }
    if (dt < 0.02) return;
    this->lastUpdateTime = current_time;

    this->timeSinceDirectionChange += dt;

    if (this->timeSinceDirectionChange >= this->directionChangeInterval) {
      this->PickNewDirection();
      this->directionChangeInterval = 2.0 + (std::rand() % 50) / 10.0;
      this->timeSinceDirectionChange = 0.0;
    }

    ignition::math::Pose3d pose = this->model->WorldPose();
    double x = pose.Pos().X();
    double y = pose.Pos().Y();

    bool bounced = false;
    
    // FIX 3: Hard clamp so they physically cannot pass the boundary
    if (x >  BOUNDARY) { x = BOUNDARY; this->vel.X(-std::abs(this->vel.X())); bounced = true; }
    if (x < -BOUNDARY) { x = -BOUNDARY; this->vel.X(std::abs(this->vel.X())); bounced = true; }
    if (y >  BOUNDARY) { y = BOUNDARY; this->vel.Y(-std::abs(this->vel.Y())); bounced = true; }
    if (y < -BOUNDARY) { y = -BOUNDARY; this->vel.Y(std::abs(this->vel.Y())); bounced = true; }

    if (bounced) {
      double angle = (std::rand() % 60 - 30) * PI / 180.0;
      double speed = OBSTACLE_SPEED;
      double vx = this->vel.X();
      double vy = this->vel.Y();
      this->vel.X(vx * cos(angle) - vy * sin(angle));
      this->vel.Y(vx * sin(angle) + vy * cos(angle));
      
      double mag = sqrt(this->vel.X()*this->vel.X() + this->vel.Y()*this->vel.Y());
      if (mag > 0) {
        this->vel.X(this->vel.X() / mag * speed);
        this->vel.Y(this->vel.Y() / mag * speed);
      }
      this->timeSinceDirectionChange = 0.0;
    }

    this->model->SetLinearVel(ignition::math::Vector3d(
      this->vel.X(), this->vel.Y(), 0.0));

    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));

    this->model->SetWorldPose(ignition::math::Pose3d(
      x, y, 0.0,
      0.0, 0.0, 0.0
    ));
  }

private:
  void PickNewDirection()
  {
    double angle = (std::rand() % 360) * PI / 180.0;
    this->vel = ignition::math::Vector3d(
      OBSTACLE_SPEED * cos(angle),
      OBSTACLE_SPEED * sin(angle),
      0.0
    );
  }

  physics::ModelPtr        model;
  physics::WorldPtr        world;
  event::ConnectionPtr     updateConnection;
  ignition::math::Vector3d vel;
  gazebo::common::Time     lastUpdateTime;
  double                   directionChangeInterval;
  double                   timeSinceDirectionChange;
};

GZ_REGISTER_MODEL_PLUGIN(Obstacles)
}
