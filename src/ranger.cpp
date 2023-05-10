#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

#define TOPIC_LASER "base_scan"
#define FRAME_LASER "laser"

using std::placeholders::_1;

StageNode::Vehicle::Ranger::Ranger(
  unsigned int id, Stg::ModelRanger * m,
  std::shared_ptr<Vehicle> & v, StageNode * n)
: id_(id), model(m), vehicle(v), node(n) {}

unsigned int StageNode::Vehicle::Ranger::id() const
{
  return id_;
}
void StageNode::Vehicle::Ranger::init(bool add_id_to_topic)
{
  model->Subscribe();
  topic_name = vehicle->name_space_ + TOPIC_LASER;
  frame_id = vehicle->name_space_ + FRAME_LASER;
  if (add_id_to_topic) {
    topic_name += std::to_string(id());
    frame_id += std::to_string(id());
  }

  pub = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
}

bool StageNode::Vehicle::Ranger::prepare_msg()
{
  if (msg) {return true;}
  if (model->GetSensors().size() > 1) {
    RCLCPP_WARN(node->get_logger(), "ROS Stage currently supports rangers with 1 sensor only.");
  }
  const Stg::ModelRanger::Sensor & sensor = model->GetSensors()[0];  // we use the first sensor data
  if (sensor.ranges.size() == 0) {return false;}

  msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->angle_min = -sensor.fov / 2.0;
  msg->angle_max = +sensor.fov / 2.0;
  msg->angle_increment = sensor.fov / (double)(sensor.sample_count - 1);
  msg->range_min = sensor.range.min;
  msg->range_max = sensor.range.max;
  msg->ranges.resize(sensor.ranges.size());
  msg->intensities.resize(sensor.intensities.size());
  msg->header.frame_id = frame_id;

  return true;
}

bool StageNode::Vehicle::Ranger::prepare_tf()
{
  if (transform) {return true;}

  transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

  Stg::Pose pose = model->GetPose();
  tf2::Quaternion quternion;
  quternion.setRPY(0.0, 0.0, pose.a);
  tf2::Transform txLaser =
    tf2::Transform(
    quternion,
    tf2::Vector3(pose.x, pose.y, vehicle->positionmodel->GetGeom().size.z + pose.z));
  *transform = create_transform_stamped(
    txLaser, node->sim_time_, vehicle->frame_id_base_link_,
    frame_id);
  if (node->use_static_transformations_) {
    vehicle->tf_static_broadcaster_->sendTransform(*transform);
  }
  return true;
}

void StageNode::Vehicle::Ranger::publish_msg()
{
  if (model->GetSensors().size() > 1) {
    RCLCPP_WARN(node->get_logger(), "ROS Stage currently supports rangers with 1 sensor only.");
  }

  // for now we access only the zeroth sensor of the ranger - good
  // enough for most laser models that have a single beam origin
  const Stg::ModelRanger::Sensor & sensor = model->GetSensors()[0];  // we use the first sensor data

  if (prepare_msg()) {
    msg->header.stamp = node->sim_time_;
    for (unsigned int i = 0; i < sensor.ranges.size(); i++) {
      msg->ranges[i] = sensor.ranges[i] + gaussRand(0.0,0.01);
      msg->intensities[i] = sensor.intensities[i];
    }
    pub->publish(*msg);
  }
}

double StageNode::Vehicle::Ranger::gaussRand(double mu, double sigma){
  static double V1, V2, S;
  static int phase = 0;
  double X;

  if ( phase == 0 ) {
    do {
      double U1 = (double)rand() / RAND_MAX;
      double U2 = (double)rand() / RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    } while(S >= 1 || S == 0);

    X = V1 * sqrt(-2 * log(S) / S);
  } else
    X = V2 * sqrt(-2 * log(S) / S);

  phase = 1 - phase;
  return (X*sigma + mu);
}

void StageNode::Vehicle::Ranger::publish_tf()
{
  if (prepare_tf()) {

    if (node->use_static_transformations_) {return;}

    // use tf publsiher only if use_static_transformations_ is false
    transform->header.stamp = node->sim_time_;
    vehicle->tf_broadcaster_->sendTransform(*transform);
  }
}
