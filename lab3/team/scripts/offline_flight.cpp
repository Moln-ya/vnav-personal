// Numerical closed-loop check using the production controller and an independent
// rigid-body plant. This is not the Unity/TESSE simulator or RViz evidence.
#include <controller_pkg/geometric_controller.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using controller_pkg::Reference;
using controller_pkg::State;

Reference referenceAt(double t, bool stationary) {
  Reference r;
  const double pi = std::acos(-1.0);
  if (stationary) {
    r.x.z() = 2.0;
    r.yaw = pi/4.0;
  } else {
    r.x << 5*std::sin(t/2), 5*std::cos(t/2), 2;
    r.v << 2.5*std::cos(t/2), -2.5*std::sin(t/2), 0;
    r.a << -1.25*std::sin(t/2), -1.25*std::cos(t/2), 0;
    r.yaw = -t/2;
  }
  return r;
}

int main(int argc, char** argv) {
  const bool stationary = argc > 1 && std::string(argv[1]) == "hover";
  const double dt = argc > 2 ? std::stod(argv[2]) : 0.001;
  const bool drag = argc > 3 && std::string(argv[3]) == "drag";
  const bool compensate = argc > 4 && std::string(argv[4]) == "compensate";
  controller_pkg::GeometricController controller;
  controller.compensate_drag = compensate;
  State state;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  const double period = 4*std::acos(-1.0);
  const double finish = stationary ? 20.0 : 3*period;
  double error_sq = 0, peak_error = 0, peak_speed = 0, map_error = 0;
  std::size_t samples = 0;
  std::ofstream csv("flight.csv");
  csv << "t,x,y,z,xd,yd,zd,error,yaw_error,rotor1,rotor2,rotor3,rotor4\n";
  Reference desired = referenceAt(0, stationary);
  int desired_tick = -1;
  for (int step = 0; step*dt < finish; ++step) {
    const double t = step*dt;
    // Match the actual trajectory publisher's 100 Hz sample-and-hold.
    const int tick = static_cast<int>(std::floor((t+1e-9)*100));
    if (tick != desired_tick) {
      desired = referenceAt(tick/100.0, stationary);
      desired_tick = tick;
    }
    const auto command = controller.compute(state, desired);
    if (!command.rotor_speeds.allFinite()) return 2;
    // Independently derive forces/moments from each rotor's position and spin.
    const double arm = 0.3/std::sqrt(2.0);
    const Eigen::Vector3d positions[4] = {{arm,arm,0}, {-arm,arm,0},
                                        {-arm,-arm,0}, {arm,-arm,0}};
    double thrust = 0;
    Eigen::Vector3d moment = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; ++i) {
      const double speed = command.rotor_speeds(i);
      const double s = speed*std::abs(speed);
      const double lift = 1e-3*s;
      thrust += lift;
      moment += positions[i].cross(Eigen::Vector3d(0,0,lift));
      moment.z() += (i%2 == 0 ? 1 : -1)*1e-5*s;
      peak_speed = std::max(peak_speed, std::abs(speed));
    }
    Eigen::Vector4d actual_wrench;
    actual_wrench << thrust, moment;
    map_error = std::max(map_error, (actual_wrench-command.wrench).norm());
    const auto ideal = referenceAt(t, stationary);
    const double error = (state.x-ideal.x).norm();
    const double yaw = std::atan2(state.R(1,0),state.R(0,0));
    const double yaw_error = std::atan2(std::sin(yaw-ideal.yaw),std::cos(yaw-ideal.yaw));
    if (t >= (stationary ? 15.0 : 2*period)) {
      error_sq += error*error;
      peak_error = std::max(peak_error,error);
      ++samples;
    }
    if (step%10 == 0) {
      csv << std::setprecision(10) << t << ',' << state.x.x() << ',' << state.x.y()
          << ',' << state.x.z() << ',' << ideal.x.x() << ',' << ideal.x.y() << ','
          << ideal.x.z() << ',' << error << ',' << yaw_error;
      for (int i = 0; i < 4; ++i) csv << ',' << command.rotor_speeds(i);
      csv << '\n';
    }
    Eigen::Vector3d acceleration = -9.81*Eigen::Vector3d::UnitZ()+thrust*state.R.col(2);
    if (drag) acceleration += controller_pkg::GeometricController::dragWorld(state);
    // Semi-implicit integration; quaternion exponential preserves SO(3).
    state.v += dt*acceleration;
    state.x += dt*state.v;
    state.omega += dt*(moment-state.omega.cross(state.omega));  // J=I, m=1
    const double rate = state.omega.norm();
    if (rate > 1e-12)
      q = (q*Eigen::Quaterniond(Eigen::AngleAxisd(dt*rate,state.omega/rate))).normalized();
    state.R = q.toRotationMatrix();
    if (!state.x.allFinite() || state.x.norm() > 1e4) return 3;
  }
  std::cout << std::setprecision(9)
            << "mode=" << (stationary ? "hover" : "circle") << " dt=" << dt
            << " drag=" << drag << " compensation=" << compensate << '\n'
            << "steady_rms_position_error_m=" << std::sqrt(error_sq/samples) << '\n'
            << "steady_max_position_error_m=" << peak_error << '\n'
            << "peak_abs_rotor_speed_rad_s=" << peak_speed << '\n'
            << "max_wrench_reconstruction_error=" << map_error << '\n';
  return 0;
}
