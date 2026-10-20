#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

// Minimum-jerk blend s(u) = 10u^3 - 15u^4 + 6u^5 on u in [0,1], with its
// derivatives with respect to time for a blend lasting T seconds.
static void minJerk(double t, double T, double& s, double& ds, double& dds) {
  const double u = std::min(std::max(t/T, 0.0), 1.0);
  s = u*u*u*(10 - 15*u + 6*u*u);
  ds = 30*u*u*(1 - u)*(1 - u)/T;
  dds = 60*u*(1 - u)*(1 - 2*u)/(T*T);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_publisher");
  ros::NodeHandle nh, pnh("~");
  bool static_pose, takeoff;
  double radius, time_scale, height, takeoff_time, hold_time, ramp_time;
  pnh.param("static_pose", static_pose, false);
  pnh.param("radius", radius, 5.0);
  pnh.param("time_scale", time_scale, 2.0);
  pnh.param("height", height, 2.0);
  // Optional smooth start: fly from the current pose to the circle start,
  // hover, then ramp the angular rate up to the official 1/time_scale.
  pnh.param("takeoff", takeoff, false);
  pnh.param("takeoff_time", takeoff_time, 6.0);
  pnh.param("hold_time", hold_time, 3.0);
  pnh.param("ramp_time", ramp_time, 6.0);
  if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(time_scale) ||
      time_scale <= 0 || !std::isfinite(height) || !(takeoff_time > 0) ||
      !(hold_time >= 0) || !(ramp_time > 0)) {
    ROS_FATAL("Invalid trajectory parameters");
    return 1;
  }
  auto pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
  tf2_ros::TransformBroadcaster br;
  ros::Rate rate(100);
  ros::Time::waitForValid();  // TESSE starts publishing /clock after initialization.
  tf2::Vector3 p0(0, radius, height);
  double yaw0 = 0.0;
  if (takeoff && !static_pose) {
    const auto state = ros::topic::waitForMessage<nav_msgs::Odometry>("current_state", nh);
    if (!state) return 0;
    tf2::fromMsg(state->pose.pose.position, p0);
    tf2::Quaternion q0;
    tf2::fromMsg(state->pose.pose.orientation, q0);
    yaw0 = tf2::getYaw(q0);
    ROS_INFO("Takeoff from (%.2f, %.2f, %.2f), yaw %.0f deg", p0.x(), p0.y(), p0.z(),
             yaw0*180/M_PI);
  }
  // Circle time t: phase = t/time_scale once the ramp is over. Without the
  // ramp it is the official reference from t=0; with it, the start transfer
  // and hover run at negative t and the ramp is centred on t=0.
  const double omega = 1.0/time_scale;
  const double lead = takeoff ? takeoff_time + hold_time + ramp_time/2 : 0.0;
  ros::Time start = ros::Time::now();
  while (ros::ok()) {
    const ros::Time now = ros::Time::now();
    if (now < start) start = now;
    const double t = (now-start).toSec() - lead;
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
    } else if (takeoff && t < -hold_time - ramp_time/2) {
      // Start transfer: straight minimum-jerk line, yaw the short way to 0.
      double s, ds, dds;
      minJerk(t + lead, takeoff_time, s, ds, dds);
      const tf2::Vector3 d = tf2::Vector3(0, radius, height) - p0;
      const double dyaw = std::atan2(std::sin(-yaw0), std::cos(-yaw0));
      p.x = p0.x() + s*d.x();  p.y = p0.y() + s*d.y();  p.z = p0.z() + s*d.z();
      v.x = ds*d.x();  v.y = ds*d.y();  v.z = ds*d.z();
      a.x = dds*d.x();  a.y = dds*d.y();  a.z = dds*d.z();
      q.setRPY(0, 0, yaw0 + s*dyaw);
      msg.velocities[0].angular.z = ds*dyaw;
    } else {
      // Phase and its derivatives. The ramp raises the rate with a smooth
      // step over ramp_time; afterwards phase = t/time_scale exactly.
      double phase = omega*t, rate_phase = omega, acc_phase = 0.0;
      if (takeoff && t < ramp_time/2) {
        const double u = std::max(0.0, (t + ramp_time/2)/ramp_time);
        phase = omega*ramp_time*(u*u*u - u*u*u*u/2);
        rate_phase = omega*u*u*(3 - 2*u);
        acc_phase = omega*6*u*(1 - u)/ramp_time;
      }
      // Preserve the official 5 m radius, 2 s time scale circular reference.
      const double c = std::cos(phase), sn = std::sin(phase);
      p.x = radius*sn;
      p.y = radius*c;
      v.x = radius*c*rate_phase;
      v.y = -radius*sn*rate_phase;
      a.x = -radius*sn*rate_phase*rate_phase + radius*c*acc_phase;
      a.y = -radius*c*rate_phase*rate_phase - radius*sn*acc_phase;
      q.setRPY(0, 0, -phase);
      msg.velocities[0].angular.z = -rate_phase;
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
