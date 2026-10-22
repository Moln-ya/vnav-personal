#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

// Part 0: publish the first gate repeatedly so the controller can keep hovering.
class SimplePlanner {
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;
  tf::TransformBroadcaster broadcaster_;
  geometry_msgs::Transform target_;
  bool ready_ = false;
  ros::Time last_publish_;
 public:
  explicit SimplePlanner(ros::NodeHandle& nh) {
    pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    sub_ = nh.subscribe("desired_traj_vertices", 1, &SimplePlanner::waypoints, this);
    timer_ = nh.createTimer(ros::Duration(0.01), &SimplePlanner::publish, this);
  }
  void waypoints(const geometry_msgs::PoseArray& msg) {
    if (msg.poses.empty()) {
      ROS_WARN("Empty gate list ignored");
      return;
    }
    const auto& p = msg.poses.front();
    tf::Quaternion q;
    tf::quaternionMsgToTF(p.orientation, q);
    if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z) || !std::isfinite(q.length2()) || q.length2() < 1e-12) {
      ROS_WARN("Invalid first gate ignored");
      return;
    }
    target_.translation.x = p.position.x;
    target_.translation.y = p.position.y;
    target_.translation.z = p.position.z;
    q.normalize();
    tf::quaternionTFToMsg(q, target_.rotation);
    ready_ = true;
  }
  void publish(const ros::TimerEvent&) {
    if (!ready_) return;
    const auto now = ros::Time::now();
    if (now == last_publish_) return;
    last_publish_ = now;
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms.push_back(target_);
    point.velocities.resize(1);       // Zero linear and angular velocity.
    point.accelerations.resize(1);    // Zero linear and angular acceleration.
    pub_.publish(point);
    tf::Transform transform;
    tf::transformMsgToTF(target_, transform);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                    "world", "desired_pose"));
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle nh;
  SimplePlanner planner(nh);
  ros::spin();
}
