#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

class WaypointFollower {
  ros::Subscriber current_state_sub_, vertices_sub_;
  ros::Publisher desired_pub_, synced_state_pub_, path_pub_;
  ros::Timer timer_;
  tf::TransformBroadcaster broadcaster_;
  Eigen::Vector3d x_ = Eigen::Vector3d::Zero();
  double yaw_ = 0.0;
  bool have_state_ = false;
  ros::Time state_received_, state_stamp_, start_time_, last_publish_;
  geometry_msgs::PoseArray pending_;
  mav_trajectory_generation::Trajectory trajectory_, yaw_trajectory_;
  double v_max_, a_max_, a_z_max_, corridor_, time_scale_, min_segment_time_, state_timeout_;
  bool level_gates_, fit_limits_, sync_to_state_;

  // Part 1.1: record the world position and heading before planning.
  void onCurrentState(const nav_msgs::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, q);
    Eigen::Vector3d next(p.x, p.y, p.z);
    if (!next.allFinite() || !std::isfinite(q.length2()) || q.length2() < 1e-12) return;
    x_ = next;
    yaw_ = tf::getYaw(q.normalized());
    have_state_ = true;
    state_received_ = ros::Time::now();
    state_stamp_ = msg.header.stamp.isZero() ? state_received_ : msg.header.stamp;
    if (trajectory_.empty() && !pending_.poses.empty()) generate(pending_);
    // The sim clock advances in uneven 20-80 ms steps, so a timer sample is often
    // 20-40 ms ahead of the state the controller pairs it with (0.16-0.32 m at
    // 8 m/s). Sampling at the state's own stamp gives the controller a matched pair.
    // The state is forwarded only after its reference, so a controller listening
    // on synced_state never pairs it with the previous sample.
    if (sync_to_state_) {
      publishAt(state_stamp_);
      synced_state_pub_.publish(msg);
    }
  }

  void onWaypoints(const geometry_msgs::PoseArray& msg) {
    if (!trajectory_.empty()) return;  // The publisher repeats this message at 5 Hz.
    if (msg.poses.empty()) {
      ROS_WARN("Empty waypoint list ignored");
      return;
    }
    for (const auto& pose : msg.poses) {
      const auto& p = pose.position;
      tf::Quaternion q;
      tf::quaternionMsgToTF(pose.orientation, q);
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(q.length2()) || q.length2() < 1e-12) {
        ROS_ERROR("Invalid waypoint list ignored");
        return;
      }
    }
    pending_ = msg;
    if (have_state_ && (ros::Time::now() - state_received_).toSec() <= state_timeout_)
      generate(pending_);
  }

  // Part 1.2: optimize xyz and unwrapped yaw with identical segment times.
  // z is solved separately so each gate can also pin vertical velocity to zero:
  // otherwise the climb into one gate carries the reference over the next one
  // (between gates 2 and 3 it peaked at z=4.46 m and hit the hall's colliders).
  void generate(const geometry_msgs::PoseArray& msg) {
    using namespace mav_trajectory_generation;
    using namespace mav_trajectory_generation::derivative_order;
    Vertex::Vector vertices, xy_vertices, z_vertices, yaw_vertices;
    Vertex start(3), start_xy(2), start_z(1), start_yaw(1);
    start.makeStartOrEnd(x_, SNAP);
    start_xy.makeStartOrEnd(Eigen::Vector2d(x_.head<2>()), SNAP);
    start_z.makeStartOrEnd(x_.z(), SNAP);
    start_yaw.makeStartOrEnd(yaw_, SNAP);
    vertices.push_back(start);
    xy_vertices.push_back(start_xy);
    z_vertices.push_back(start_z);
    yaw_vertices.push_back(start_yaw);
    double last_yaw = yaw_;
    for (size_t i = 0; i < msg.poses.size(); ++i) {
      const auto& pose = msg.poses[i];
      const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
      Vertex vertex(3), xy_vertex(2), z_vertex(1), yaw_vertex(1);
      tf::Quaternion q;
      tf::quaternionMsgToTF(pose.orientation, q);
      const double raw_yaw = tf::getYaw(q.normalized());
      const double yaw = last_yaw + std::atan2(std::sin(raw_yaw - last_yaw),
                                              std::cos(raw_yaw - last_yaw));
      if (i + 1 == msg.poses.size()) {
        vertex.makeStartOrEnd(position, SNAP);
        xy_vertex.makeStartOrEnd(Eigen::Vector2d(position.head<2>()), SNAP);
        z_vertex.makeStartOrEnd(position.z(), SNAP);
        yaw_vertex.makeStartOrEnd(yaw, SNAP);
      } else {
        vertex.addConstraint(POSITION, position);
        xy_vertex.addConstraint(POSITION, Eigen::Vector2d(position.head<2>()));
        z_vertex.addConstraint(POSITION, position.z());
        if (level_gates_) z_vertex.addConstraint(VELOCITY, 0.0);
        yaw_vertex.addConstraint(ORIENTATION, yaw);
      }
      vertices.push_back(vertex);
      xy_vertices.push_back(xy_vertex);
      z_vertices.push_back(z_vertex);
      yaw_vertices.push_back(yaw_vertex);
      last_yaw = yaw;
    }
    auto times = estimateSegmentTimes(vertices, v_max_, a_max_);
    for (double& time : times) time = std::max(min_segment_time_, time * time_scale_);
    if (!solve(xy_vertices, z_vertices, yaw_vertices, times)) return;
    if (fit_limits_) {
      // Stretching a segment by s scales its v by 1/s and a by 1/s^2. Moving each
      // segment toward its own binding limit lets the open middle of the course run
      // fast instead of every segment inheriting the scale of the hardest one.
      // Unequal times also reshape the path (one try bulged 3.8 m into a wall), so
      // a step that takes any segment `corridor` further from its chord than the
      // uniform-time path is rejected and the step size halved. The fastest
      // allocation seen, after uniform scaling to the limits, is kept; the uniform
      // allocation itself is the first candidate.
      const std::vector<double> base_deviation = chordDeviations();
      std::vector<double> best = times;
      double best_total = std::numeric_limits<double>::infinity();
      double step = 0.5;
      for (int iteration = 0; iteration < 100 && step > 0.02; ++iteration) {
        std::vector<double> loads;
        for (const auto& segment : trajectory_.segments()) loads.push_back(segmentLoad(segment));
        const double s = *std::max_element(loads.begin(), loads.end());
        if (!std::isfinite(s) || s <= 1e-3) break;  // A motionless path keeps its times.
        double total = 0.0;
        for (double time : times) total += time * s;
        if (total < best_total) {
          best_total = total;
          best = times;
          for (double& time : best) time *= s;
        }
        const std::vector<double> accepted = times;
        for (size_t i = 0; i < times.size(); ++i)
          if (std::isfinite(loads[i]) && loads[i] > 1e-3)
            times[i] = std::max(min_segment_time_, times[i] * std::pow(loads[i], step));
        if (!solve(xy_vertices, z_vertices, yaw_vertices, times)) return;
        const std::vector<double> deviation = chordDeviations();
        for (size_t i = 0; i < deviation.size(); ++i) {
          if (deviation[i] > base_deviation[i] + corridor_) {
            step *= 0.5;
            times = accepted;
            if (!solve(xy_vertices, z_vertices, yaw_vertices, times)) return;
            break;
          }
        }
      }
      if (std::isfinite(best_total)) {
        times = best;
        for (double& time : times) time = std::max(min_segment_time_, time);
        if (!solve(xy_vertices, z_vertices, yaw_vertices, times)) return;
      }
    }
    double v_peak, a_peak;
    trajectory_.computeMaxVelocityAndAcceleration(&v_peak, &a_peak);
    start_time_ = state_stamp_;  // Time origin of the start vertex x_.
    ROS_INFO("Generated trajectory: %zu gates, %.3f s, peak |v|=%.2f m/s |a|=%.2f m/s^2",
             msg.poses.size(), trajectory_.getMaxTime(), v_peak, a_peak);
    publishPath();
  }

  // The whole planned path, latched for RViz.
  void publishPath() {
    using namespace mav_trajectory_generation::derivative_order;
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = start_time_;
    for (double t = 0.0;; t = std::min(t + 0.05, trajectory_.getMaxTime())) {
      const Eigen::Vector3d p = trajectory_.evaluate(t, POSITION);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.header.stamp = start_time_ + ros::Duration(t);
      pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.position.z = p.z();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_trajectory_.evaluate(t, ORIENTATION)(0));
      path.poses.push_back(pose);
      if (t >= trajectory_.getMaxTime()) break;
    }
    path_pub_.publish(path);
  }

  // Time stretch that would put this segment exactly at its tightest limit. The
  // vertical limit is separate because a rotor cannot pull the drone down faster
  // than gravity: a sharp climb into a level gate otherwise flips it (84 deg seen).
  double segmentLoad(const mav_trajectory_generation::Segment& segment) const {
    using namespace mav_trajectory_generation::derivative_order;
    double load = 0.0;
    for (int k = 0; k <= 100; ++k) {
      const double t = segment.getTime() * k / 100.0;
      const Eigen::VectorXd v = segment.evaluate(t, VELOCITY);
      const Eigen::VectorXd a = segment.evaluate(t, ACCELERATION);
      load = std::max({load, v.norm() / v_max_, std::sqrt(a.norm() / a_max_),
                       std::sqrt(std::abs(a(2)) / a_z_max_)});
    }
    return load;
  }

  // Largest distance of each segment from the straight line joining its ends.
  std::vector<double> chordDeviations() const {
    using namespace mav_trajectory_generation::derivative_order;
    std::vector<double> deviations;
    for (const auto& segment : trajectory_.segments()) {
      const Eigen::Vector3d a = segment.evaluate(0.0, POSITION);
      const Eigen::Vector3d b = segment.evaluate(segment.getTime(), POSITION);
      const Eigen::Vector3d chord = b - a;
      double deviation = 0.0;
      for (int k = 1; k < 100; ++k) {
        const Eigen::Vector3d p = segment.evaluate(segment.getTime() * k / 100.0, POSITION);
        const double u = chord.squaredNorm() > 1e-12 ?
            std::max(0.0, std::min(1.0, (p - a).dot(chord) / chord.squaredNorm())) : 0.0;
        deviation = std::max(deviation, (p - a - u * chord).norm());
      }
      deviations.push_back(deviation);
    }
    return deviations;
  }

  bool solve(const mav_trajectory_generation::Vertex::Vector& xy_vertices,
             const mav_trajectory_generation::Vertex::Vector& z_vertices,
             const mav_trajectory_generation::Vertex::Vector& yaw_vertices,
             const std::vector<double>& times) {
    using namespace mav_trajectory_generation;
    const int N = 10;
    PolynomialOptimization<N> xy_opt(2), z_opt(1), yaw_opt(1);
    Trajectory xy, z;
    trajectory_.clear();
    if (!xy_opt.setupFromVertices(xy_vertices, times, derivative_order::SNAP) || !xy_opt.solveLinear() ||
        !z_opt.setupFromVertices(z_vertices, times, derivative_order::SNAP) || !z_opt.solveLinear() ||
        !yaw_opt.setupFromVertices(yaw_vertices, times, derivative_order::SNAP) || !yaw_opt.solveLinear()) {
      ROS_ERROR("Trajectory optimization failed");
      return false;
    }
    xy_opt.getTrajectory(&xy);
    z_opt.getTrajectory(&z);
    yaw_opt.getTrajectory(&yaw_trajectory_);
    if (!xy.getTrajectoryWithAppendedDimension(z, &trajectory_)) {
      trajectory_.clear();
      ROS_ERROR("Trajectory optimization failed");
      return false;
    }
    return true;
  }

  // Part 1.3: sample elapsed ROS time and keep holding the final rest pose.
  // The timer is the fallback when states stop arriving.
  void publish(const ros::TimerEvent& event) {
    const auto now = ros::Time::now();
    if (sync_to_state_ && have_state_ &&
        (now - state_received_).toSec() < 5.0 * (event.current_expected - event.last_expected).toSec())
      return;
    publishAt(now);
  }

  void publishAt(const ros::Time& stamp) {
    if (trajectory_.empty()) return;
    if (stamp <= last_publish_) return;
    last_publish_ = stamp;
    using namespace mav_trajectory_generation::derivative_order;
    const double t = std::max(0.0, std::min((stamp - start_time_).toSec(),
                                          trajectory_.getMaxTime()));
    const Eigen::Vector3d p = trajectory_.evaluate(t, POSITION);
    const Eigen::Vector3d v = trajectory_.evaluate(t, VELOCITY);
    const Eigen::Vector3d a = trajectory_.evaluate(t, ACCELERATION);
    const double yaw = yaw_trajectory_.evaluate(t, ORIENTATION)(0);
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.time_from_start = ros::Duration(t);
    point.transforms.resize(1);
    point.velocities.resize(1);
    point.accelerations.resize(1);
    auto& transform = point.transforms[0];
    transform.translation.x = p.x(); transform.translation.y = p.y(); transform.translation.z = p.z();
    transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
    point.velocities[0].linear.x = v.x();
    point.velocities[0].linear.y = v.y();
    point.velocities[0].linear.z = v.z();
    point.velocities[0].angular.z = yaw_trajectory_.evaluate(t, VELOCITY)(0);
    point.accelerations[0].linear.x = a.x();
    point.accelerations[0].linear.y = a.y();
    point.accelerations[0].linear.z = a.z();
    point.accelerations[0].angular.z = yaw_trajectory_.evaluate(t, ACCELERATION)(0);
    desired_pub_.publish(point);
    tf::Transform tf_pose;
    tf::transformMsgToTF(transform, tf_pose);
    broadcaster_.sendTransform(tf::StampedTransform(tf_pose, stamp, "world", "desired_pose"));
    ROS_INFO_THROTTLE(2.0, "Trajectory %.1f%%", 100.0 * t / trajectory_.getMaxTime());
  }
 public:
  explicit WaypointFollower(ros::NodeHandle& nh) {
    ros::NodeHandle pnh("~");
    double hz;
    pnh.param("v_max", v_max_, 3.0);
    pnh.param("a_max", a_max_, 2.0);
    pnh.param("a_z_max", a_z_max_, 5.0);
    pnh.param("corridor", corridor_, 0.5);
    pnh.param("time_scale", time_scale_, 1.0);
    pnh.param("min_segment_time", min_segment_time_, 0.5);
    pnh.param("state_timeout", state_timeout_, 0.5);
    pnh.param("publish_rate", hz, 100.0);
    pnh.param("level_gates", level_gates_, true);
    pnh.param("fit_limits", fit_limits_, true);
    pnh.param("sync_to_state", sync_to_state_, true);
    for (double value : {v_max_, a_max_, a_z_max_, corridor_, time_scale_, min_segment_time_, state_timeout_, hz})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Trajectory parameters must be positive and finite");
    current_state_sub_ = nh.subscribe("current_state", 1, &WaypointFollower::onCurrentState, this,
                                       ros::TransportHints().tcpNoDelay());
    vertices_sub_ = nh.subscribe("desired_traj_vertices", 1, &WaypointFollower::onWaypoints, this);
    desired_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
    synced_state_pub_ = nh.advertise<nav_msgs::Odometry>("synced_state", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("desired_path", 1, true);
    timer_ = nh.createTimer(ros::Duration(1.0 / hz), &WaypointFollower::publish, this);
  }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_generation_node");
  ros::NodeHandle nh;
  try { WaypointFollower follower(nh); ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
}
