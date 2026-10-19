#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <controller_pkg/geometric_controller.h>
#include <stdexcept>

class controllerNode {
  ros::NodeHandle nh, pnh;
  // Part 1: retain handles for the subscriptions, publisher, and timer.
  ros::Subscriber desired_sub, current_sub;
  ros::Publisher rotor_pub;
  ros::Timer timer;
  controller_pkg::GeometricController controller;
  controller_pkg::State state;
  controller_pkg::Reference reference;
  bool have_current = false, have_desired = false;
  ros::Time current_received, desired_received;
  double hz, state_timeout, desired_timeout;

 public:
  controllerNode() : pnh("~") {
    // Part 6: gains have defaults and are configurable in the launch file.
    pnh.param("kx", controller.gains.kx, 6.0);
    pnh.param("kv", controller.gains.kv, 4.7);
    pnh.param("kr", controller.gains.kr, 64.0);
    pnh.param("komega", controller.gains.komega, 16.0);
    pnh.param("compensate_drag", controller.compensate_drag, false);
    pnh.param("hz", hz, 1000.0);
    pnh.param("state_timeout", state_timeout, 0.5);
    pnh.param("desired_timeout", desired_timeout, 0.5);
    for (double value : {controller.gains.kx, controller.gains.kv,
                         controller.gains.kr, controller.gains.komega,
                         hz, state_timeout, desired_timeout}) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Gains, frequency and timeouts must be positive and finite");
    }
    // Part 2: official topic names and message types; callbacks use latest state.
    desired_sub = nh.subscribe("desired_state", 1, &controllerNode::onDesiredState, this);
    current_sub = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this,
                               ros::TransportHints().tcpNoDelay());
    rotor_pub = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
    timer = nh.createTimer(ros::Duration(1.0/hz), &controllerNode::controlLoop, this);
    ROS_INFO("Gains kx=%g kv=%g kr=%g komega=%g; control rate=%g Hz",
             controller.gains.kx, controller.gains.kv,
             controller.gains.kr, controller.gains.komega, hz);
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg) {
    // Part 3: guard variable-length arrays before accessing element zero.
    if (msg.transforms.empty() || msg.velocities.empty() || msg.accelerations.empty()) {
      have_desired = false;
      ROS_WARN_THROTTLE(1.0, "Desired state requires a transform, velocity and acceleration");
      return;
    }
    controller_pkg::Reference next;
    tf2::fromMsg(msg.transforms[0].translation, next.x);
    tf2::fromMsg(msg.velocities[0].linear, next.v);
    tf2::fromMsg(msg.accelerations[0].linear, next.a);
    tf2::Quaternion q;
    tf2::fromMsg(msg.transforms[0].rotation, q);
    if (!next.x.allFinite() || !next.v.allFinite() || !next.a.allFinite() ||
        !std::isfinite(q.length2()) || q.length2() < 1e-12) {
      have_desired = false;
      ROS_WARN_THROTTLE(1.0, "Ignoring invalid desired state");
      return;
    }
    q.normalize();
    next.yaw = tf2::getYaw(q);
    reference = next;
    desired_received = ros::Time::now();
    have_desired = true;
  }

  void onCurrentState(const nav_msgs::Odometry& msg) {
    // Part 4: TESSE publishes both twist vectors in WORLD coordinates.
    // This differs from the usual Odometry child_frame_id twist convention.
    controller_pkg::State next;
    tf2::fromMsg(msg.pose.pose.position, next.x);
    tf2::fromMsg(msg.twist.twist.linear, next.v);
    Eigen::Quaterniond q;
    tf2::fromMsg(msg.pose.pose.orientation, q);
    Eigen::Vector3d omega_world;
    tf2::fromMsg(msg.twist.twist.angular, omega_world);
    if (!next.x.allFinite() || !next.v.allFinite() || !omega_world.allFinite() ||
        !q.coeffs().allFinite() || q.squaredNorm() < 1e-12) {
      have_current = false;
      ROS_WARN_THROTTLE(1.0, "Ignoring invalid current state");
      return;
    }
    next.R = q.normalized().toRotationMatrix();
    next.omega = next.R.transpose()*omega_world;
    state = next;
    current_received = ros::Time::now();
    have_current = true;
  }

  void controlLoop(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double state_age = (now-current_received).toSec();
    const double desired_age = (now-desired_received).toSec();
    if (!have_current || !have_desired || state_age < 0.0 || desired_age < 0.0 ||
        state_age > state_timeout || desired_age > desired_timeout) return;

    // Part 5: equations, desired SO(3) attitude and signed allocation are in
    // geometric_controller.h so the same implementation can run without ROS.
    const auto cmd = controller.compute(state, reference);
    if (!cmd.rotor_speeds.allFinite()) {
      ROS_ERROR_THROTTLE(1.0, "Nonfinite rotor command suppressed");
      return;
    }
    mav_msgs::Actuators msg;
    msg.header.stamp = now;
    msg.header.frame_id = "base_link_gt";
    msg.angular_velocities.assign(cmd.rotor_speeds.data(), cmd.rotor_speeds.data()+4);
    rotor_pub.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  try {
    controllerNode node;
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Controller initialization failed: %s", e.what());
    return 1;
  }
  return 0;
}
