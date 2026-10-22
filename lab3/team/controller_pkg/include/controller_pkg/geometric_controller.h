#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace controller_pkg {

struct State {
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // body -> world
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();  // body frame
};

struct Reference {
  Eigen::Vector3d x = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

struct Gains {
  double kx = 6.0;
  double kv = 4.7;
  double kr = 64.0;
  double komega = 16.0;
};

struct Command {
  Eigen::Matrix3d Rd;
  Eigen::Vector4d wrench;  // thrust, body Mx, My, Mz
  Eigen::Vector4d rotor_speeds;  // signed rad/s, rotor order 1,2,3,4
};

// Lab 3 skeleton model constants; Unity's internal inertia may differ.
class GeometricController {
 public:
  Gains gains;
  bool compensate_drag = false;
  const double m = 1.0, g = 9.81, d = 0.3, cf = 1e-3, cd = 1e-5;
  const Eigen::Matrix3d J = Eigen::Matrix3d::Identity();

  GeometricController() {
    // rho1=(+a,+a,0), rho2=(-a,+a,0), rho3=(-a,-a,0), rho4=(+a,-a,0).
    // rho x (0,0,cf*s): Mx=y*cf*s, My=-x*cf*s.
    // TESSE yaw signs: +,-,+,-. s_i = omega_i*abs(omega_i).
    const double a = d / std::sqrt(2.0);
    allocation_ << cf, cf, cf, cf,
                   a*cf, a*cf, -a*cf, -a*cf,
                   -a*cf, a*cf, a*cf, -a*cf,
                   cd, -cd, cd, -cd;
    F2W_ = allocation_.inverse();
  }

  const Eigen::Matrix4d& allocation() const { return allocation_; }

  static Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
  }

  static Eigen::Vector3d dragWorld(const State& state) {
    const Eigen::Vector3d vb = state.R.transpose() * state.v;
    const Eigen::Vector3d drag_b =
        -Eigen::Vector3d(0.1, 0.1, 0.2).cwiseProduct(vb.cwiseProduct(vb.cwiseAbs()));
    return state.R * drag_b;
  }

  Command compute(const State& state, const Reference& ref) const {
    Command cmd;
    // Part 5.1: equations (6),(7).
    const Eigen::Vector3d ex = state.x - ref.x;
    const Eigen::Vector3d ev = state.v - ref.v;
    const Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();

    // Part 5.2: ROS uses upward z and +f*R*e3 thrust, unlike the paper.
    Eigen::Vector3d A = -gains.kx*ex - gains.kv*ev + m*g*e3 + m*ref.a;
    if (compensate_drag) A -= dragWorld(state);
    // With zero requested force the thrust direction is undetermined.
    const Eigen::Vector3d b3d = A.norm() > 1e-9 ? A.normalized() : e3;
    Eigen::Vector3d b1c(std::cos(ref.yaw), std::sin(ref.yaw), 0.0);
    Eigen::Vector3d b2d = b3d.cross(b1c);
    if (b2d.norm() < 1e-9) {
      // Pick a perpendicular heading if the requested heading is parallel to thrust.
      b1c = b3d.unitOrthogonal();
      b2d = b3d.cross(b1c);
    }
    b2d.normalize();
    cmd.Rd.col(0) = b2d.cross(b3d);
    cmd.Rd.col(1) = b2d;
    cmd.Rd.col(2) = b3d;

    // Part 5.3: the handout permits Omega_d=0 and dot(Omega_d)=0.
    const Eigen::Vector3d er =
        0.5 * vee(cmd.Rd.transpose()*state.R - state.R.transpose()*cmd.Rd);
    const Eigen::Vector3d eomega = state.omega;

    // Part 5.4: equations (15),(16), adapted to upward z.
    cmd.wrench(0) = A.dot(state.R.col(2));
    cmd.wrench.tail<3>() = -gains.kr*er - gains.komega*eomega
                          + state.omega.cross(J*state.omega);

    // Parts 5.5,5.6: invert the wrench map and preserve reversible rotor signs.
    const Eigen::Vector4d signed_squared_speeds = F2W_ * cmd.wrench;
    for (int i = 0; i < 4; ++i) {
      const double s = signed_squared_speeds(i);
      cmd.rotor_speeds(i) = std::copysign(std::sqrt(std::abs(s)), s);
    }
    return cmd;
  }

 private:
  Eigen::Matrix4d allocation_, F2W_;
};
}  // namespace controller_pkg
