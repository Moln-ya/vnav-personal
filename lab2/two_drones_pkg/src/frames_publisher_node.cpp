#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>

class FramesPublisherNode {
 private:
  ros::NodeHandle nh;
  ros::Time startup_time;

  ros::Timer heartbeat;

  // Broadcast dynamic transforms to the ROS TF tree.
  tf2_ros::TransformBroadcaster tf_broadcaster;

 public:
  FramesPublisherNode() {
    // Record the time at which the node starts.
    startup_time = ros::Time::now();

    // Call onPublish() every 0.02 s, i.e. at 50 Hz.
    heartbeat =
        nh.createTimer(ros::Duration(0.02),
                       &FramesPublisherNode::onPublish,
                       this);

    heartbeat.start();
  }

  void onPublish(const ros::TimerEvent&) {
    // Time elapsed since the node was started.
    double time = (ros::Time::now() - startup_time).toSec();

    geometry_msgs::TransformStamped AV1World;
    geometry_msgs::TransformStamped AV2World;

    ros::Time current_time = ros::Time::now();

    // ============================================================
    // AV1
    //
    // Position:
    //   x = cos(t)
    //   y = sin(t)
    //   z = 0
    //
    // Orientation:
    //   roll  = 0
    //   pitch = 0
    //   yaw   = t
    //
    // This makes the AV1 y-axis tangent to its circular trajectory.
    // ============================================================

    AV1World.header.stamp = current_time;
    AV1World.header.frame_id = "world";
    AV1World.child_frame_id = "av1";

    AV1World.transform.translation.x = std::cos(time);
    AV1World.transform.translation.y = std::sin(time);
    AV1World.transform.translation.z = 0.0;

    tf2::Quaternion q_av1;
    q_av1.setRPY(0.0, 0.0, time);
    q_av1.normalize();

    AV1World.transform.rotation.x = q_av1.x();
    AV1World.transform.rotation.y = q_av1.y();
    AV1World.transform.rotation.z = q_av1.z();
    AV1World.transform.rotation.w = q_av1.w();

    // ============================================================
    // AV2
    //
    // Position:
    //   x = sin(t)
    //   y = 0
    //   z = cos(2t)
    //
    // Orientation is not important for this exercise, therefore
    // an identity rotation is used.
    // ============================================================

    AV2World.header.stamp = current_time;
    AV2World.header.frame_id = "world";
    AV2World.child_frame_id = "av2";

    AV2World.transform.translation.x = std::sin(time);
    AV2World.transform.translation.y = 0.0;
    AV2World.transform.translation.z = std::cos(2.0 * time);

    AV2World.transform.rotation.x = 0.0;
    AV2World.transform.rotation.y = 0.0;
    AV2World.transform.rotation.z = 0.0;
    AV2World.transform.rotation.w = 1.0;

    // Publish both transforms.
    tf_broadcaster.sendTransform(AV1World);
    tf_broadcaster.sendTransform(AV2World);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "frames_publisher_node");

  FramesPublisherNode node;

  ros::spin();

  return 0;
}
