#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_publisher");
  ros::NodeHandle nh, pnh("~");
  bool static_pose;
  double radius, time_scale, height;
  pnh.param("static_pose", static_pose, false);
  pnh.param("radius", radius, 5.0);
  pnh.param("time_scale", time_scale, 2.0);
  pnh.param("height", height, 2.0);
  if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(time_scale) ||
      time_scale <= 0 || !std::isfinite(height)) {
    ROS_FATAL("Invalid trajectory parameters");
    return 1;
  }
  auto pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
  tf2_ros::TransformBroadcaster br;
  ros::Rate rate(100);
  ros::Time::waitForValid();  // TESSE starts publishing /clock after initialization.
  ros::Time start = ros::Time::now();
  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    if (now < start) start = now;
    const double t = (now-start).toSec();
    const double phase = t/time_scale;
    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.velocities.resize(1);
    msg.accelerations.resize(1);
    auto& p = msg.transforms[0].translation;
    auto& v = msg.velocities[0].linear;
    auto& a = msg.accelerations[0].linear;
    p.z = height;
    tf2::Quaternion q;
    if (static_pose) {
      q.setRPY(0, 0, std::acos(-1.0)/4.0);
    } else {
      // Preserve the official 5 m radius, 2 s time scale circular reference.
      p.x = radius*std::sin(phase);
      p.y = radius*std::cos(phase);
      v.x = radius*std::cos(phase)/time_scale;
      v.y = -radius*std::sin(phase)/time_scale;
      a.x = -radius*std::sin(phase)/(time_scale*time_scale);
      a.y = -radius*std::cos(phase)/(time_scale*time_scale);
      q.setRPY(0, 0, -phase);
      msg.velocities[0].angular.z = -1.0/time_scale;
    }
    msg.transforms[0].rotation = tf2::toMsg(q);
    msg.time_from_start = ros::Duration(t);
    pub.publish(msg);
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "world";
    transform.child_frame_id = "av-desired";
    transform.transform = msg.transforms[0];
    br.sendTransform(transform);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
