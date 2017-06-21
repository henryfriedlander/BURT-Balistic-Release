/*  Copyright 2016 Barrett Technology <support@barrett.com>
 *
 *  This file is part of proficio_toolbox.
 *
 *  This version of proficio_toolbox is free software: you can redistribute it
 *  and/or modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This version of proficio_toolbox is distributed in the hope that it will be
 *  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this version of proficio_toolbox.  If not, see
 *  <http://www.gnu.org/licenses/>.
 */

/**
 *  @file haptic_world.cpp
 *
 *  This is a demonstration of the capabilities of the Proficio robot for
 *  rehabilitation. In this demo, are a variety of objects that provide haptic
 *  and visual feedback when interacted with.
 *
 *  This file includes the haptics and logic behind the game. It must be run
 *  concurrently with the corresponding python visualization. To run the demo,
 *  run the following commands in two separate terminals:
 *
 *    ./haptic_world <IP address>
 *    python haptic_world_visualization.py <IP address>
 *
 *  By default, the IP address is 127.0.0.1 for both commands
 *
 *  User assistance is available in this demo. Gravity assist aids the user in
 *  supporting the weight of their own arm. The degree of assistance can be
 *  adjusted with the following key presses:
 *
 *  key          | action
 *  ------------ | -----------------------
 *  <up arrow>   | Increase gravity assist
 *  <down arrow> | Decrease gravity assist
 *  <delete>     | Turn off gravity assist
 *
 *  NOTICE: This program is for demonstration purposes only.
 *  It is not approved for clinical use.
 */

#include "haptic_world.h"

#include <signal.h>  // signal, raise, sig_atomic_t
#include <string.h>

// Networking
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

// Barrett Library
#include <barrett/math.h>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/config.h>

#include <proficio/systems/utilities.h>

#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>

/* Constants describing out environment in meters */
const double kBackWallX = 0.3;
const double kFrontWallX = 0.7;
const double kLeftWallY = -0.25;
const double kRightWallY = 0.25;
const double kTopWallZ = 0.5;
const double kBottomWallZ = 0.0;

/* Button */
const double kButtonRadius = 0.15;
const double kButtonDepth = 0.01;

/* Proxy Parameters */
const double kProxyRadius = 0.03;

/* Magnetic field */
const double kMagneticFieldSize = 0.01;

/* Ball Parameters */
const double kBallStartX = 0.45;  // 0.25;
const double kBallStartY = -0.125;  //-0.2;
const double kBallStartZ = 0.25;
const double kBallRadius = 0.075;
const double kBallMass = 50.0;
const double kGravityForce = 0.00015;
const double kAirDamping = 0.0005;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(10);

const char* remote_host = NULL;
cp_type proxy_pos;
cv_type proxy_vel;
cp_type ball_pos;
cv_type ball_vel;
v_type msg_tmp;
bool button_pressed;
barrett::systems::ExposedOutput<v_type> message;
// double stiffness; // Spring Force (Kp)
// double damping; // Damping Force (Kd)

/* Set the IP address */
bool validate_args(int argc, char** argv) {
  if (argc != 2) {
    remote_host = "127.0.0.1";
    printf("Defaulting to 127.0.0.1\n");
  } else {
    remote_host = argv[1];
  }
  return true;
}

cf_type hapticCalc(boost::tuple<cp_type, cv_type> haptics_tuple) {
  cf_type net_force;
  cp_type wam_pos = haptics_tuple.get<0>();
  cv_type wam_vel = haptics_tuple.get<1>();

  proxy_pos = wam_pos;
  proxy_vel = wam_vel;

  double stiffness = 1000.0;  // Spring Force (Kp)
  double damping = 15.0;      // Damping Force (Kd)

  /* Ball Calculations */
  cp_type proxy_ball_vec = proxy_pos - ball_pos;

  if (proxy_ball_vec.norm() < (kBallRadius + kProxyRadius)) {
    proxy_pos = ball_pos + (kBallRadius + kProxyRadius) * proxy_ball_vec /
                               proxy_ball_vec.norm();
    // const double timestep = product_manager_.getExecutionManager()->getPeriod()
    // printf("Timestep: %d\n", timestep);
    double timestep = 0.002; //500Hz
    // changes the balls velocity when touched by the proxy accordingly in all
    // three dimensions
    ball_vel[0] =
        ball_vel[0] +
        (-(stiffness * (proxy_pos[0] - wam_pos[0])) / kBallMass) * timestep;
    ball_vel[1] =
        ball_vel[1] +
        (-(stiffness * (proxy_pos[1] - wam_pos[1])) / kBallMass) * timestep;
    ball_vel[2] =
        ball_vel[2] +
        (-(stiffness * (proxy_pos[2] - wam_pos[2])) / kBallMass) * timestep -
        kGravityForce;

  } else {
    ball_vel[2] -= kGravityForce;  // gravity compensation on the ball
    ball_vel -= kAirDamping * ball_vel;  // damping due to air friction
  }

  net_force[0] = stiffness * (proxy_pos[0] - wam_pos[0]);
  net_force[1] = stiffness * (proxy_pos[1] - wam_pos[1]);
  net_force[2] = stiffness * (proxy_pos[2] - wam_pos[2]);

  // Check for a collision with the ball and right wall using balls radius
  if (ball_pos[1] > kRightWallY - kBallRadius) {
    ball_vel[1] = -ball_vel[1];
  }

  // Check for a collision with the ball and button / left wall using balls
  // radius
  if (ball_pos[1] < kLeftWallY + kButtonDepth + kBallRadius &&
      ball_pos[0] <
          kBackWallX + ((kFrontWallX - kBackWallX) / 2) + kButtonRadius &&
      ball_pos[0] >
          kBackWallX + ((kFrontWallX - kBackWallX) / 2) - kButtonRadius &&
      ball_pos[2] <
          kBottomWallZ + ((kTopWallZ - kBottomWallZ) / 2) + kButtonRadius &&
      ball_pos[2] >
          kBottomWallZ + ((kTopWallZ - kBottomWallZ) / 2) - kButtonRadius) {
    ball_vel[1] = -ball_vel[1];
  } else if (ball_pos[1] < kLeftWallY + kBallRadius)
    ball_vel[1] = -ball_vel[1];

  // Check for a collision with the ball and ceiling using balls radius
  if (ball_pos[2] > kTopWallZ - kBallRadius) {
    ball_vel[2] = -ball_vel[2];
  }

  // Check for a collision with the ball and floor using balls radius
  if (ball_pos[2] < kBottomWallZ + kBallRadius) {
    ball_vel[2] = -ball_vel[2];
  }

  // Check for a collision with the ball and front wall using balls radius
  if (ball_pos[0] > kFrontWallX - kBallRadius) {
    ball_vel[0] = -ball_vel[0];
  }

  // Check for a collision with the ball and back wall using balls radius
  if (ball_pos[0] < kBackWallX + kBallRadius) {
    ball_vel[0] = -ball_vel[0];
  }

  ball_pos = ball_pos + 0.001 * ball_vel;

  // Check for a collision with the back wall - Sticky
  if (wam_pos[0] < kBackWallX + kProxyRadius) {
    proxy_pos[0] = kBackWallX + kProxyRadius;
    net_force[0] +=
        0.6 * (stiffness * (proxy_pos[0] - wam_pos[0]));  // Spring Portion
    net_force[0] -= damping * wam_vel[0];  // Damping portion
    net_force[1] -= damping * wam_vel[1];
    net_force[2] -= damping * wam_vel[2];
  }

  // Check for a collision with the front wall - Nothing Keep person in box.
  if (wam_pos[0] > kFrontWallX - kProxyRadius) {
    proxy_pos[0] = kFrontWallX - kProxyRadius;
    net_force[0] += (stiffness * (proxy_pos[0] - wam_pos[0]));  // Spring
                                                                // Portion
    net_force[0] -= damping * wam_vel[0];  // Damping portion
  }

  // Check for a collision with the right wall - Magnetic
  if (wam_pos[1] > kRightWallY - kProxyRadius - kMagneticFieldSize) {
    proxy_pos[1] = kRightWallY - kProxyRadius;
    net_force[1] += (stiffness * (proxy_pos[1] - wam_pos[1]));  // Spring
                                                                // Portion
    net_force[1] -= damping * wam_vel[1];
  }

  button_pressed = false;
  // Check for a collision with the left button / WALL
  if (wam_pos[1] < kLeftWallY + kButtonDepth + kProxyRadius &&
      wam_pos[0] <
          kBackWallX + ((kFrontWallX - kBackWallX) / 2) + kButtonRadius &&
      wam_pos[0] >
          kBackWallX + ((kFrontWallX - kBackWallX) / 2) - kButtonRadius &&
      wam_pos[2] <
          kBottomWallZ + ((kTopWallZ - kBottomWallZ) / 2) + kButtonRadius &&
      wam_pos[2] >
          kBottomWallZ + ((kTopWallZ - kBottomWallZ) / 2) - kButtonRadius) {
    proxy_pos[1] = kLeftWallY + kButtonDepth + kProxyRadius;
    if (wam_pos[1] - proxy_pos[1] < -0.01) {
      proxy_pos[1] = kLeftWallY + kProxyRadius;
      net_force[1] +=
          (stiffness * (proxy_pos[1] - wam_pos[1]));  // Spring Portion
      //      net_force[1] -= damping * wam_vel[1]; // Damping portion

      button_pressed = true;
    } else {
      net_force[1] +=
          (stiffness * (proxy_pos[1] - wam_pos[1]));  // Spring Portion
      //      net_force[1] -= damping * wam_vel[1]; // Damping portion
    }
  } else if (wam_pos[1] < kLeftWallY + kProxyRadius) {
    proxy_pos[1] = kLeftWallY + kProxyRadius;
    net_force[1] += (stiffness * (proxy_pos[1] - wam_pos[1]));  // Spring
                                                                // Portion
    //    net_force[1] -= damping * wam_vel[1]; // Damping portion
  }

  // Check for a collision with the ceiling - Slippery inverse damping
  if (wam_pos[2] > kTopWallZ - kProxyRadius) {
    proxy_pos[2] = kTopWallZ - kProxyRadius;
    net_force[2] += (stiffness * (proxy_pos[2] - wam_pos[2]));  // Spring
                                                                // Portion
    net_force[2] -= damping * wam_vel[2];  // Damping portion
    net_force[0] += 0.5 * damping * wam_vel[0];
    net_force[1] += 0.25 * damping * wam_vel[1];
  }

  // Check for a collision with the floor.
  if (wam_pos[2] < kBottomWallZ + kProxyRadius) {
    proxy_pos[2] = kBottomWallZ + kProxyRadius;
    net_force[2] += (stiffness * (proxy_pos[2] - wam_pos[2]));
    net_force[2] -= damping * wam_vel[2];
    if (sin(500 * wam_pos[1]) >= 0)
      net_force[1] =
          stiffness * (proxy_pos[1] - wam_pos[1]) - damping * wam_vel[1] * 2;
    /* creates the illusion of stripes on the wall by multiplying
    the position by sin(.5*position), and turns on the parallel
    damping when it is above 0 and turns it off when it is less than zero */
  }
  return net_force;
}

enum ContactState {
  PLAYING,
  TARING,
  QUIT
} curState = PLAYING,
  lastState = PLAYING;

template <size_t DOF>
class HapticsDemo {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

 protected:
  /* Wam and ProductManager Instances Passed into Class */
  barrett::systems::Wam<DOF>& wam;
  barrett::ProductManager& product_manager_;

  /* Network Parameters */
  struct sockaddr_in si_server;
  int port, sock, i, slen;
  char* buf;
  char* srv_addr;

  jt_type jtLimits;
  barrett::systems::TupleGrouper<cp_type, cv_type> tuple_grouper;

  barrett::systems::ToolForceToJointTorques<DOF> cf_tf2jt;

  cf_type net_force;
  jp_type init_joint_pos;
  cf_type curForces;

  barrett::systems::Callback<boost::tuple<cp_type, cv_type>, cf_type> haptics;
  proficio::systems::JointTorqueSaturation<DOF> jtsat;
  proficio::systems::UserGravityCompensation<DOF>* user_grav_comp_;
  barrett::systems::Summer<jt_type, 3> jtSum;
  v_type dampingConstants;
  jv_type velocityLimits;
  proficio::systems::JointVelocitySaturation<DOF> velsat;

  barrett::systems::modXYZ<cp_type> invpos;
  barrett::systems::modXYZ<cf_type> invforce;
  barrett::systems::modXYZ<cv_type> invvel;
  barrett::systems::modXYZ<cv_type> invvelSat;

  jv_type jvFiltFreq;
  cv_type cvFiltFreq;
  barrett::systems::FirstOrderFilter<jv_type> jvFilter;
  barrett::systems::FirstOrderFilter<cv_type> cvFilter;

 public:
  bool ftOn;
  double sumForces;

  HapticsDemo(barrett::systems::Wam<DOF>& wam_arm, barrett::ProductManager& pm,
              proficio::systems::UserGravityCompensation<DOF>* ugc)
      : wam(wam_arm),
        product_manager_(pm),
        slen(sizeof(si_server)),
        jtLimits(45.0),
        haptics(hapticCalc),
        jtsat(jtLimits),
        user_grav_comp_(ugc),
        jtSum("+++"),
        dampingConstants(20.0),
        velocityLimits(1.4),
        velsat(dampingConstants, velocityLimits),
        jvFiltFreq(20.0),
        cvFiltFreq(20.0),
        sumForces(0.0) {
    dampingConstants[2] = 10.0;
    dampingConstants[0] = 30.0;
    velsat.setDampingConstant(dampingConstants);
    invpos.negX();
    invpos.negY();
    invpos.xOffset(1);
    invforce.negX();
    invforce.negY();
    invvel.negX();
    invvel.negY();
    invvelSat.negX();
    invvelSat.negY();
    jvFilter.setLowPass(jvFiltFreq);
    cvFilter.setLowPass(cvFiltFreq);
  }
  ~HapticsDemo() {}

  bool init(Config side);
  bool setupNetworking();
  void displayEntryPoint();
  void connectForces();
  void visualizationThread();
};

/* Initialization Method */
template <size_t DOF>
bool HapticsDemo<DOF>::init(Config side) {
  wam.gravityCompensate();
  product_manager_.getSafetyModule()->setVelocityLimit(1.5);
  product_manager_.getSafetyModule()->setTorqueLimit(2.5);

  if (side == LEFT) {
    init_joint_pos << -0.1, 0, -1.5;
  } else if (side == RIGHT) {
    init_joint_pos << -0.1, 0, 1.5;
  }
  wam.moveTo(init_joint_pos);

  for (int i = 0; i < 3; i++) {
    proxy_pos[i] = 0.0;
    proxy_vel[i] = 0.0;
    ball_vel[i] = 0.0;
  }
  ball_pos[0] = kBallStartX;
  ball_pos[1] = kBallStartY;
  ball_pos[2] = kBallStartZ;
  for (int j = 0; j < 10; j++) {
    if (j < 3)
      msg_tmp[j] = proxy_pos[j];
    else if (j < 6)
      msg_tmp[j] = proxy_vel[j - 3];
    else if (j < 9)
      msg_tmp[j] = ball_pos[j - 6];
    else
      msg_tmp[j] = (double)button_pressed;
  }
  button_pressed = false;
  wam.idle();
  return true;
}

/* */
template <size_t DOF>
bool HapticsDemo<DOF>::setupNetworking() {
  buf = new char[1024];
  srv_addr = new char[16];
  memset(srv_addr, 0, 16);
  port = 5555;
  if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    printf("Failure creating the socket");
    return false;
  }
  memcpy(srv_addr, remote_host, strlen(remote_host));
  memset((char*)&si_server, 0, sizeof(si_server));
  si_server.sin_family = AF_INET;
  si_server.sin_port = htons(port);
  if (inet_aton(srv_addr, &si_server.sin_addr) == 0) {
    printf("inet_aton() failed - EXITING\n");
    return false;
  }
  return true;
}

/*  */
template <size_t DOF>
void HapticsDemo<DOF>::connectForces() {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

  barrett::systems::connect(wam.kinematicsBase.kinOutput, cf_tf2jt.kinInput);
  barrett::systems::connect(wam.jpOutput, user_grav_comp_->input);
  barrett::systems::connect(wam.jvOutput, jvFilter.input);
  barrett::systems::connect(jvFilter.output, velsat.input);
  // barrett::systems::connect(velsat.output, invvelSat.input);
  barrett::systems::connect(wam.toolPosition.output, invpos.input);
  barrett::systems::connect(invpos.output, tuple_grouper.getInput<0>());
  barrett::systems::connect(haptics.output, invforce.input);
  barrett::systems::connect(invforce.output, cf_tf2jt.input);
  barrett::systems::connect(wam.toolVelocity.output, cvFilter.input);
  barrett::systems::connect(cvFilter.output, invvel.input);
  barrett::systems::connect(invvel.output, tuple_grouper.getInput<1>());
  barrett::systems::connect(tuple_grouper.output, haptics.input);
  barrett::systems::connect(cf_tf2jt.output, jtSum.getInput(0));
  barrett::systems::connect(user_grav_comp_->output, jtSum.getInput(1));
  barrett::systems::connect(velsat.output, jtSum.getInput(2));
  // barrett::systems::connect(invvelSat.output, jtSum.getInput(2));
  barrett::systems::connect(jtSum.output, jtsat.input);
  barrett::systems::connect(jtsat.output, wam.input);
}

/*  */
template <size_t DOF>
void HapticsDemo<DOF>::visualizationThread() {
  while (curState == PLAYING) {
    sumForces = 0.0;
    for (int i = 0; i < 3; i++) sumForces += fabs(curForces[i]);
    for (int j = 0; j < 10; j++) {
      if (j < 3)
        msg_tmp[j] = proxy_pos[j];
      else if (j < 6)
        msg_tmp[j] = proxy_vel[j - 3];
      else if (j < 9)
        msg_tmp[j] = ball_pos[j - 6];
      else
        msg_tmp[j] = (double)button_pressed;
    }
    message.setValue(msg_tmp);
    barrett::btsleep(0.01);
  }
}

/*  */
template <size_t DOF>
int proficio_main(int argc, char** argv, barrett::ProductManager& product_manager_,
                  barrett::systems::Wam<DOF>& wam, const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  std::string fname = "calibration_data/wam3/";
  if (side == LEFT) {  // Left Config
    fname = fname + "LeftConfig.txt";
  } else if (side == RIGHT) {
    fname = fname + "RightConfig.txt";
  }
  proficio::systems::UserGravityCompensation<DOF> user_grav_comp_(
      barrett::EtcPathRelative(fname).c_str());
  user_grav_comp_.setGainZero();
  HapticsDemo<DOF> haptics_demo(wam, product_manager_, &user_grav_comp_);
  if (!haptics_demo.setupNetworking()) {
    return 1;
  }
  if (!haptics_demo.init(side)) return 1;
  // instantiate Systems
  NetworkHaptics<DOF> nh(product_manager_.getExecutionManager(), remote_host,
                         &user_grav_comp_);
  message.setValue(msg_tmp);
  barrett::systems::forceConnect(message.output, nh.input);
  haptics_demo.connectForces();
  boost::thread visualizeThread(&HapticsDemo<DOF>::visualizationThread,
                                &haptics_demo);
  bool running = true;
  haptics_demo.ftOn = false;
  while (running) {
    switch (curState) {
      case QUIT:
        product_manager_.getPuck(1)->setProperty(product_manager_.getPuck(1)->getBus(), 1, 8, 3);
        product_manager_.getPuck(2)->setProperty(product_manager_.getPuck(2)->getBus(), 2, 8, 3);
        product_manager_.getPuck(3)->setProperty(product_manager_.getPuck(3)->getBus(), 3, 8, 3);
        barrett::systems::disconnect(wam.input);
        running = false;
        break;
      case PLAYING:
        lastState = PLAYING;
        barrett::btsleep(0.1);
        break;
      default:
        break;
    }
  }
  wam.moveHome();
  printf("\n\n");
  return 0;
}
