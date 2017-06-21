/* Copyright 2016 Barrett Technology support@barrett.com
 *
 * This file is part of proficio_toolbox.
 *
 * This version of proficio_toolbox is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of proficio_toolbox is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of proficio_toolbox.  If not, see
 * http://www.gnu.org/licenses/.
 */

/** @file ball_popping.cpp
 *
 *  This is a demonstration of the capabilities of the Proficio robot for
 *  rehabilitation. In this game, spherical targets appear in three
 *  dimensional space, and the user is asked to move the robot to these
 *  locations.
 *
 *  This file includes the haptics and logic behind the game. It must be run
 *  concurrently with the corresponding python visualization. To run the demo,
 *  run the following commands in two separate terminals:
 *
 *    ./ball_popping <IP address> <time in seconds>
 *    python ball_popping_visualization.py <IP address>
 *
 *  By default, the IP address is 127.0.0.1 for both commands (running the
 *  haptics and the visualization on the same PC). The time in seconds is the
 *  length of the game before it resets. The default time is 40 seconds.
 *
 *  This demo includes a workspace calibration routine that runs at the
 *  beginning of the demo. It ensures that targets always appear within the
 *  user's range of motion.
 *
 *  User assistance is available in this demo. Gravity assist aids the user in
 *  supporting the weight of their own arm. Movement assistance aids the user
 *  in moving towards the target. The degree of assistance can be adjusted with
 *  the following key presses:
 *
 *  key          | action
 *  ------------------------------------------------
 *  <up arrow>   | Increase gravity assist
 *  <down arrow> | Decrease gravity assist
 *  <delete>     | Turn off gravity assist
 *  1            | Movement assistance speed low
 *  2            | Movement assistance speed medium
 *  3            | Movement assistance speed high
 *  4            | Movement assistance delay long
 *  5            | Movement assistance delay medium
 *  6            | Movement assistance delay short
 *  7            | Movement assistance force low
 *  8            | Movement assistance force high
 *  9            | Movement assistance force off
 *
 *  NOTICE: This program is for demonstration purposes only. It is not approved
 *  for clinical use.
 */

#include "network_haptics.h"

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <signal.h>
#include <fstream>
#include <math.h>
#include <syslog.h>

#include <barrett/config.h>
#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems.h>
#include <barrett/units.h>
#include <proficio/systems/utilities.h>

#define BARRETT_SMF_VALIDATE_ARGS
//#define NO_CONTROL_PENDANT

#include <proficio/standard_proficio_main.h>

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
BARRETT_UNITS_TYPEDEFS(15);  // defines v_type to have length 15

const char* remoteHost = NULL;
double kp = 3e3;
double kd = 3e1;
double segment_s = 40;  // time (s) of game, can be changed by command line arg

bool validate_args(int argc, char** argv) {
  switch (argc) {
    case 3:
      segment_s = atof(argv[2]);
    case 2:
      remoteHost = argv[1];
      break;

    default:
      remoteHost = "127.0.0.1";
      printf("Using default host 127.0.0.1\n");
      break;
  }

  printf("Gains: kp = %f; kd = %f\n", kp, kd);
  return true;
}

namespace ball_popping {
bool game_exit = false;
void exit_pgm(int signum) { game_exit = true; }
}  // namespace ball_popping

namespace ball_popping {
v_type msg_tmp;

barrett::systems::ExposedOutput<v_type> message;

template <size_t DOF>
class SolidHapticBall : public barrett::systems::HapticBall {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  
 public:
  barrett::systems::System::Output<cf_type> output;
  barrett::systems::System::Input<boost::tuple<double, double, double, double, double, double, double> >
    ball_input;
  barrett::systems::System::Input<cv_type> wamVel_input;
  bool calibrate, new_pos;
  cp_type cart_pos;
  double BALLR;

  explicit SolidHapticBall(proficio::systems::UserGravityCompensation<DOF> &ugc,
      const cp_type& center, double radius,
      const Config& side,
      const std::string& sysName = "HapticBall") :
    kSide(side),
    kMagFieldDistance(0.02),  // The distance of magnetic field around the floor
    kFloorWidth(0.01),  // Width of the floor
    kFloorPositionZ(0.05),  // Floor's absolute Z position
    kToolRadius(0.05),  // Radius of the end point. should match with that in the python client
    // Game (python client) workspace. Should match with that in the python client
    kGameWorkspaceMinX(-1),
    kGameWorkspaceMaxX(0.5),
    kGameWorkspaceMinY(-1.4),
    kGameWorkspaceMaxY(0.6),
    kGameWorkspaceMinZ(0.06),  // Floor + wall size
    kGameWorkspaceMaxZ(1.55),
    // The actual workspace's original range which would be expanded upon calibration
    user_min_x_(0.25),
    user_max_x_(0.35),
    user_min_y_(0.15),
    user_max_y_(0.25),
    user_min_z_(0.15),
    user_max_z_(0.25),
    HapticBall(center, radius, sysName),
    gc(&ugc),
    output(this, &outputValue),
    ball_input(this),
    wamVel_input(this),
    kWaitTime(1.0)
    {
      initialize(center);
      readConfig();
    }

  virtual ~SolidHapticBall() {mandatoryCleanUp();}

  cp_type getUserMin() {
    cp_type user_min(user_min_x_, user_min_y_, user_min_z_);
    return user_min;
  }

  cp_type getUserMax() {
    cp_type user_max(user_max_x_, user_max_y_, user_max_z_);
    return user_max;
  }

  void readConfig() {
    char fname[] = "config.txt";
    // Provide the absolute location of the file from which the constants are to be read
    std::ifstream myfile(fname);
    std::string line;
    size_t i=0;
    size_t size = 4;
    std::vector<double> a(size, 0);
    if (myfile.is_open()) {
      while(getline(myfile,line) && i < size) {
        a[i] = atof(line.c_str());
        i++;
      }
      myfile.close();
    }
    ss_kp = a[0];
    ss_kd = a[1];
    mag_ball_kp = a[2];
  }

  inline bool inEllipsoid(cp_type pos){
    double r1, r2, r3;
    r1 = (user_max_x_ - user_min_x_) / 2;
    r2 = (user_max_y_ - user_min_y_) / 2;
    r3 = (user_max_z_ - user_min_z_) / 2;
    cp_type center;
    center << user_min_x_ + r1, user_min_y_ + r2, user_min_z_ + r3;
    double dist = pow((pos[0] - center[0])/r1,2) + pow((pos[1] - center[1])/r2,2) + pow((pos[2] - center[2])/r3,2);
    if (dist < 1)
      return true;
    return false;
  }

  inline bool isDistanced(cp_type pos){
    pos = scale(pos);
    if ((pos - prev_ballCenter).norm() >= 0.75)
      return true;
    return false;
  }

 protected:
  barrett::systems::System::Output<cf_type>::Value* outputValue;

  const Config kSide;

  const double kMagFieldDistance;  // The distance of magnetic field around the floor
  const double kFloorWidth;  // Width of the floor
  const double kFloorPositionZ;  // Floor's absolute Z position
  const double kToolRadius;  // Radius of the end point. should match with that in the python client

  // Game (python client) workspace
  const double kGameWorkspaceMinX;
  const double kGameWorkspaceMaxX;
  const double kGameWorkspaceMinY;
  const double kGameWorkspaceMaxY;
  const double kGameWorkspaceMinZ;
  const double kGameWorkspaceMaxZ;

  // The actual workspace's original range which would be expanded upon calibration
  double user_min_x_;
  double user_max_x_;
  double user_min_y_;
  double user_max_y_;
  double user_min_z_;
  double user_max_z_;

  const double kWaitTime;  // Wait time between balls
  cp_type toolCenter, ballCenter, proxyCenter, shrinkSphereOrigin,
          shrinkSphere_ctr, prev_ballCenter;
  cf_type cfSum, direction;
  cv_type wamVelocity, ballVelocity;
  double stiffness, damping, stiff, damp, scale_x,
         shrinkSphere_r, counter, ss_kp, ss_kd, mag_ball_kp, max_kp, min_kp, max_kd,
         min_kd, MAG_FLOOR, speed, game_mode, timer, force_delay;
  bool prevState, posChanged, inside, cbt_init, attached, wait,
       game_pause, next_seg, it_wait;
  int score;
  proficio::systems::UserGravityCompensation<DOF> * gc;

  void initialize(const cp_type& center){
    ballCenter = center;
    stiffness = 1000;
    damping = 2;
    shrinkSphere_r = 1.5;
    counter = 0;
    ss_kp = 220;
    ss_kd = 1;
    mag_ball_kp = 1400;
    max_kp = 700;
    min_kp = 50;
    max_kd = 4;
    min_kd = 1;
    speed = 0.0005;
    game_mode = -1;
    prevState = false;
    posChanged = false;
    calibrate = false;
    inside = false;
    cbt_init = false;
    attached = false;
    wait = false;
    game_pause = false;
    score = 0;
    new_pos = false;
    next_seg = true;
    BALLR = 0.5;
    it_wait = false;
    force_delay = 0.03;
    shrinkSphere_ctr.setZero();
    shrinkSphereOrigin.setZero();
    ballVelocity.setZero();
    prev_ballCenter.setZero();
    counter = segment_s;
    MAG_FLOOR = kFloorPositionZ + kFloorWidth;
    timer = kWaitTime;
    scale_x = (toolCenter[0] - kGameWorkspaceMinX) * (1/(kGameWorkspaceMaxX - kGameWorkspaceMinX));
    stiff = min_kp + ((scale_x) * (max_kp - min_kp));
    damp = min_kd + ((scale_x) * (max_kd - min_kd));
  }

  /** Converts the coordinates from the game workspace to the user workspace. */
  cp_type descale(cp_type val) {
    val[0] = user_min_x_
      + ((val[0] - kGameWorkspaceMinX) * static_cast<double>(user_max_x_ - user_min_x_)
          / (kGameWorkspaceMaxX - kGameWorkspaceMinX));
    val[1] = user_min_y_
      + ((val[1] - kGameWorkspaceMinY) * static_cast<double>(user_max_y_ - user_min_y_)
          / (kGameWorkspaceMaxY - kGameWorkspaceMinY));
    val[2] = user_min_z_
      + ((val[2] - kGameWorkspaceMinZ) * static_cast<double>(user_max_z_ - user_min_z_)
          / (kGameWorkspaceMaxZ - kGameWorkspaceMinZ));

    return val;
  }

  /** Converts the coordinates from the user workspace to the game workspace. */
  cp_type scale(cp_type val) {
    val[0] = kGameWorkspaceMinX
      + ((val[0] - user_min_x_) * static_cast<double>(kGameWorkspaceMaxX - kGameWorkspaceMinX)
          / (user_max_x_ - user_min_x_));
    val[1] = kGameWorkspaceMinY
      + ((val[1] - user_min_y_) * static_cast<double>(kGameWorkspaceMaxY - kGameWorkspaceMinY)
          / (user_max_y_ - user_min_y_));
    val[2] = kGameWorkspaceMinZ
      + ((val[2] - user_min_z_) * static_cast<double>(kGameWorkspaceMaxZ - kGameWorkspaceMinZ)
          / (user_max_z_ - user_min_z_));

    return val;
  }

  cf_type scale(boost::tuple<cf_type, double> t) {
    return t.get<0>() * t.get<1>();
  }

  virtual void operate() {
    // Read the Inputs
    toolCenter = input.getValue();
    wamVelocity = wamVel_input.getValue();
    const double& ballRadius = ball_input.getValue().get<0>();
    BALLR = ballRadius;

    /* Indicates if the Sphere Center and the Tool Position are to be scaled to the Client's workspace or not
     * 1 = Scale the Tool Center. This scaling would be done only when no forces are acting on the Robot
     */
    double scl = ball_input.getValue().get<1>();

    /* Indicates the User's gravity compensation gain
     * 1 = Increment the gain
     * 2 = Decrement the gain
     * 3 = Set the gain as 0
     */
    double gain = ball_input.getValue().get<2>();

    /* Indicates the Forces that needs to be activated based on the part of the game that is activated
     * 0 = No forces. This is generally active till the user moves the tool position into a safety sphere to activate the forces
     * 1 = Just the floor and the wall close to the user are active. This is before the workspace calibration
     * 2 = The front wall and the floor are active. This is while calibrating the workspace.
     * 3 = The actual game. The sphere with magnetic force, 6 walls, shrinking sphere are all active.
     * 4 = New session of the game. Resets the parameters and sets the game mode back to 3.
     * 5 = Quit the game
     */
    game_mode = ball_input.getValue().get<4>();
    cfSum.setZero(); // Reset the forces at each iteration
    if(gain == 1){
      gc->incrementGain();
    }
    else if(gain == 2){
      gc->decrementGain();
    }
    else if(gain == 3){
      gc->setGainZero();
    }
    if(game_mode == 0){ // This game mode enters into none of the loops below that generates forces
      if (scl == 1) // This combination is triggered if we need to have 0 forces after defining a workspace.
        toolCenter = scale(toolCenter);
    }
    if(game_mode == 1){ // Just the magnetic Floor
      initialize(ballCenter);
      // Floor forces
      if (toolCenter[2] < MAG_FLOOR + (kToolRadius)) {
        proxyCenter[2] = MAG_FLOOR + (kToolRadius);
        cfSum[2] += ((2 + (scale_x * 575)) * (proxyCenter[2] - toolCenter[2]));
      }
      // Front Wall forces
      if(toolCenter[0] > kGameWorkspaceMaxX - kToolRadius) {
        proxyCenter[0] = kGameWorkspaceMaxX - kToolRadius;
        cfSum[0] += (75 * (proxyCenter[0] - toolCenter[0]));
      }
    }
    if (game_mode == 2 ) { // Workspace calibration
      if(!cbt_init){ // This condition is to initialize the 3d workspace around the current position of the Wam's end point
        user_max_x_ = toolCenter[0] + 0.05;
        user_min_x_ = toolCenter[0] - 0.05;
        user_max_y_ = toolCenter[1] + 0.05;
        user_min_y_ = toolCenter[1] - 0.05;
        user_min_z_ = MAG_FLOOR;
        user_max_z_ = toolCenter[2] + 0.05;
        cbt_init = true;
      }
      else{ // If the workspace is already initialized, start exapnding them
        user_min_x_ = toolCenter[0] < user_min_x_ ? toolCenter[0]:user_min_x_;
        if (toolCenter[0] > user_max_x_ && user_max_x_ < kGameWorkspaceMaxX){ // Limit the workspace not to extend beyond the front wall
          user_max_x_ = toolCenter[0];
        }
        if (kSide == LEFT) {
          if (toolCenter[1] < user_min_y_){ // This change is to account for the kinematic constraints in the left trip when the third joint is closed to its 0
            user_min_y_ = toolCenter[1];
          }
          user_max_y_ = toolCenter[1] > user_max_y_ ? toolCenter[1]:user_max_y_;
        } else {
          user_min_y_ = toolCenter[1] < user_min_y_ ? toolCenter[1]:user_min_y_;
          if (toolCenter[1] > user_max_y_ && toolCenter[1] < 0.2){
            user_max_y_ = toolCenter[1];
          }
        }
        user_max_z_ = toolCenter[2] > user_max_z_ ? toolCenter[2]:user_max_z_;

      }
      // Floor forces
      if (toolCenter[2] < MAG_FLOOR + (kToolRadius)) {
        proxyCenter[2] = MAG_FLOOR + (kToolRadius);
        cfSum[2] += ((2 + (scale_x * 575)) * (proxyCenter[2] - toolCenter[2]));
      }
      // Front Wall forces
      if(toolCenter[0] > kGameWorkspaceMaxX - kToolRadius) {
        proxyCenter[0] = kGameWorkspaceMaxX - kToolRadius;
        cfSum[0] += (75 * (proxyCenter[0] - toolCenter[0]));
      }

    }

    if(game_mode == 4){ // New Session
      // Reset the game parameters
      next_seg = true;
      counter = segment_s;
      score = 0;
      // Reset the mode to game
      game_mode = 3;
    }
    if(game_mode == 3){ // The game
      /* The rate at which the shrinking sphere's radius reduces at each loop
       * Possible options are 0.001, 0.0005, 0.00025
       */
      speed = ball_input.getValue().get<3>();
      /* The time taken for the onset of the shrinking sphere to come in contact with the tool position. Defines the edge offset of starting radius of the shrinking sphere
       * Possible options are 1, 0.5, 0.1
       */
      force_delay = ball_input.getValue().get<5>();
      /* The force with which the shrinking sphere pushes the tool position inside. Defines the gains used in the calculation.
       * Possible options are 500 and 0
       */
      ss_kp = ball_input.getValue().get<6>();
      if(next_seg && !it_wait) { // After attaching to a sphere and between the time a new sphere appears
        if (new_pos){ // If the main thread has generated a new random position for the sphere center
          ballCenter = cart_pos;
          ballCenter = scale(ballCenter);
          toolCenter = scale(toolCenter);
          attached = false;
          new_pos = false;
          next_seg = false; // Reset the parameters
          wait = false;
          shrinkSphere_r = (ballCenter - toolCenter).norm() + force_delay;
          toolCenter = descale(toolCenter);
        }
        else {
          wait = true; // Pause the game till we get the new parameters
          counter = counter - 0.002;
          // counter = counter + 0.002;
          // game_pause = true;
        }
      }
      toolCenter = scale(toolCenter);
      if(!wait){ // If the game is not waiting decrease the timer
        counter = counter - 0.002;

      }
      game_pause = counter > 0 ? false : true; // Once the timer hits 0 pause the game. This marks the new session
      shrinkSphere_ctr = ballCenter;
      // shrink the radius of the sphere
      shrinkSphere_r = (shrinkSphere_r > 0) ? (shrinkSphere_r - speed) : 0;
      // Update the ball's parameters
      this->setCenter(ballCenter);
      this->setRadius(ballRadius);

      // Find the location of the tool's center relative to the ball
      error = toolCenter - ballCenter;
      double mag = error.norm();// Distance between the tool and the ball
      double depth = mag - r;// The distance at which the tool resides with respect to the ball's radius
      // Unit vector from the ball's center towards the tool's center
      direction = error / mag;
      proxyCenter = toolCenter;
      // If the tool is outside and within the magnetic field
      if (depth <= kMagFieldDistance + kToolRadius && depth > 0 && !game_pause && !wait && !it_wait) {
        proxyCenter = ballCenter + ((ballRadius + kToolRadius) * direction);  // The position along the direction of the tool on the surface of the ball

        cfSum[0] += mag_ball_kp * (proxyCenter[0] - toolCenter[0]);
        cfSum[1] += mag_ball_kp * (proxyCenter[1] - toolCenter[1]);
        cfSum[2] += mag_ball_kp * (proxyCenter[2] - toolCenter[2]);
      }
      if ( depth <= kToolRadius && !wait && !game_pause && !it_wait) {  // If the tool is going inside the surface of the ball
        attached = true; // This segment has ended
        next_seg = true; // It is time to reset the ball center to a new position and start it again
        it_wait = true;
        score ++;
      }
      // Constructing the floor on either sides
      // Back Wall
      if(toolCenter[0] < kGameWorkspaceMinX + kToolRadius) {
        proxyCenter[0] = kGameWorkspaceMinX + kToolRadius;
        cfSum[0] += (75 * (proxyCenter[0] - toolCenter[0]));
      }
      // Front Wall
      if(toolCenter[0] > kGameWorkspaceMaxX - kToolRadius) {
        proxyCenter[0] = kGameWorkspaceMaxX - kToolRadius;
        cfSum[0] += (75 * (proxyCenter[0] - toolCenter[0]));
      }
      // Left Wall
      if(toolCenter[1] < kGameWorkspaceMinY + kToolRadius + 0.03) {
        proxyCenter[1] = kGameWorkspaceMinY + kToolRadius + 0.03;
        cfSum[1] += ((2 + (scale_x * 75)) * (proxyCenter[1] - toolCenter[1]));
      }
      // Right Wall
      if(toolCenter[1] > kGameWorkspaceMaxY - kToolRadius - 0.03) {
        proxyCenter[1] = kGameWorkspaceMaxY - kToolRadius - 0.03;
        cfSum[1] += ((2 + (scale_x * 75)) * (proxyCenter[1] - toolCenter[1]));
      }
      // Top Wall
      if(toolCenter[2] > kGameWorkspaceMaxZ - kToolRadius - 0.03) {
        proxyCenter[2] = kGameWorkspaceMaxZ - kToolRadius - 0.03;
        cfSum[2] += ((2 + (scale_x * 75)) * (proxyCenter[2] - toolCenter[2]));
      }
      if ( (shrinkSphere_r - mag) < (kToolRadius) &&
          (shrinkSphere_r - mag) > 0 && !game_pause && !wait && !it_wait) {  // The tool is touching the shrinking sphere's surface from inside

        // Apply force 0 when just touching or inside sphereRad
        // Apply increasing force when between touching and proxy center
        // Apply large force when proxy center is at sphereRad
        // Apply force 0 when proxy center is beyond sphereRad
        proxyCenter = ballCenter + (shrinkSphere_r - kToolRadius) * direction;// On the inner surface of the shrinking sphere
        cfSum[0] += ss_kp * (proxyCenter[0] - toolCenter[0]);
        cfSum[1] += ss_kp * (proxyCenter[1] - toolCenter[1]);
        cfSum[2] += ss_kp * (proxyCenter[2] - toolCenter[2]);
      }

      // After each ball is reached wait for kWaitTime seconds before starting again
      if (it_wait)
        timer = timer - 0.002;
      if (timer <= 0){
        it_wait = false;
        timer = kWaitTime;
      }
      prev_ballCenter = ballCenter;

      // Floor calculations
      if (toolCenter[2] < MAG_FLOOR + (kToolRadius)) {
        proxyCenter[2] = MAG_FLOOR + (kToolRadius);
        cfSum[2] += ((2 + (scale_x * 175)) * (proxyCenter[2] - toolCenter[2]));
      }
    }
    // If exit is signalled set the global variable as true
    else if(game_mode == 5)
      game_exit = true;

    double gcal_gain = gc->getGain(); // Read the current User's gravity calibration gain
    // Convert the ball and the tool's positions to the client's frame
    /*
     *  Update the message with the wam position, ball position and
     *   an indication of whether the ball is attached
     */
    for (int j = 0; j < 15; j++) {
      if (j < 3)
        msg_tmp[j] = toolCenter[j];
      else if (j < 4)
        msg_tmp[j] = static_cast<double>(attached);
      else if (j < 5)
        msg_tmp[j] = shrinkSphere_r;
      else if (j < 6)
        msg_tmp[j] = score;
      else if (j < 7)
        msg_tmp[j] = static_cast<double>(game_pause);
      else if(j < 10)
        msg_tmp[j] = ballCenter[j-7];
      else if(j < 11)
        msg_tmp[j] = counter;
      else if(j < 12)
        msg_tmp[j] = static_cast<double>(it_wait);
      else if(j < 13)
        msg_tmp[j] = gcal_gain;
      else if(j < 14)
        msg_tmp[j] = static_cast<double>(kSide == LEFT);
      else if(j < 15)
        msg_tmp[j] = static_cast<double>(kSide == LEFT);
    }
    outputValue->setData(&cfSum);
    message.setValue(msg_tmp);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(SolidHapticBall);
};

template<size_t DOF>
void init(barrett::systems::Wam<DOF>& wam) {
  cp_type tp = wam.getToolPosition();
  cp_type center;
  center << 0.35, 0.6, 0.0;
  for (int j = 0; j < 7; j++) {
    if (j < 3)
      msg_tmp[j] = tp[j];
    else if (j < 6)
      msg_tmp[j] = center[j - 3];  // Indicates that the ball is not attached to the tool
    else
      msg_tmp[j] = 0;
  }
}
}  // namespace ball_popping

template<size_t DOF>
int proficio_main(int argc, char** argv, barrett::ProductManager& product_manager,
    barrett::systems::Wam<DOF>& wam, const Config& side) {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  wam.gravityCompensate();
  barrett::SafetyModule* safety_module = product_manager.getSafetyModule();
  barrett::SafetyModule::PendantState pendant_state;
  safety_module->getPendantState(&pendant_state);
  if (DOF != 3){
    printf("This program is supported only for the Proficio robot");
    return 0;
  }
  // Catch kill signals if possible for a graceful exit. This is intended for
  // use with the Proficio GUI, which currently calls this program between
  // games and kills this program before starting a new game.
  signal(SIGINT, ball_popping::exit_pgm);
  signal(SIGTERM, ball_popping::exit_pgm);
  signal(SIGKILL, ball_popping::exit_pgm);

  // Create a temporary file to indicate that the program has started. This
  // serves as a flag to the GUI so it knows when to start the visualization.
  std::ofstream start_flag_file;
  start_flag_file.open("/tmp/bpstart");
  start_flag_file.close();

  // Assign configuration-dependent variables
  std::string fname="calibration_data/wam3/";  // User grav comp config file
  double j3_sign;  // Sign of joint 3 position range
  if (side == LEFT) {
    fname = fname+"LeftConfig.txt";
    j3_sign = -1.0;
  } else if (side == RIGHT) {
    fname = fname+"RightConfig.txt";
    j3_sign = 1.0;
  } else {
    printf("No known robot found ");
    return 0;
  }

  // Get the joint limits.
  // @TODO(ab): Get joint limits from a config file.
  const cp_type kJointMax(0.56, 0.96, std::max(j3_sign*2.84, j3_sign*0.40));
  const cp_type kJointMin(-1.01, -0.96, std::min(j3_sign*2.84, j3_sign*0.40));

  proficio::systems::UserGravityCompensation<DOF> gc(barrett::EtcPathRelative(fname).c_str());
  jp_type rand_ang;

  struct bt_kinematics * kin;
  config_setting_t * world;

  libconfig::Setting& wamSetting = product_manager.getConfig().lookup(product_manager.getWamDefaultConfigPath());
  bt_kinematics_create(&kin, wamSetting["kinematics"].getCSetting(), DOF);
  world = config_setting_get_member(wamSetting["kinematics"].getCSetting(),"world_to_base");

  proficio::ForwardKinematics<DOF> FwKin(*kin, world);

  jp_type jp;
  jp << 0, 0, -1;
  cp_type cp;
  cp << -0.00207499, 0.288004, 0.319999;

  // instantiate Systems
  // Invert the X,Y axes for the Proficio and add an offset
  barrett::systems::modXYZ<cp_type> modcp;
  modcp.negX();
  modcp.negY();
  modcp.xOffset(1);
  // Invert the cartesian forces and the velocities for the Proficio
  barrett::systems::modXYZ<cf_type> modforce;
  modforce.negX();
  modforce.negY();
  barrett::systems::modXYZ<cv_type> invvel;
  invvel.negX();
  invvel.negY();

  NetworkHaptics nh(product_manager.getExecutionManager(), remoteHost);
  barrett::systems::FirstOrderFilter<cv_type> fof;
  const cv_type kVelocityFilterFrequency(20, 20, 20);
  fof.setLowPass(kVelocityFilterFrequency);
  ball_popping::init(wam);
  const cp_type kStartingBallCenter(0.4, -0.3, 0);
  ball_popping::SolidHapticBall<DOF> shb(gc, kStartingBallCenter, 0.12, side);
  barrett::systems::ToolForceToJointTorques<DOF> cf_tf2jt;
  barrett::systems::Summer<jt_type, 3> jtSum("+++");
  ball_popping::message.setValue(ball_popping::msg_tmp);

  // Joint Torques Saturation
  jt_type jtLimits(35.0);
  proficio::systems::JointTorqueSaturation<DOF> jtsat(jtLimits);

  // Joint Velocities Saturation
  v_type dampingConstants(20.0);
  dampingConstants[2] = 10.0;
  dampingConstants[0] = 30.0;
  jv_type velocityLimits(1.4);
  proficio::systems::JointVelocitySaturation<DOF> velsat(dampingConstants,
      velocityLimits);
  jv_type jvFiltFreq(20.0);
  barrett::systems::FirstOrderFilter<jv_type> jvFilter;
  jvFilter.setLowPass(jvFiltFreq);

  barrett::systems::connect(wam.jvOutput, jvFilter.input);
  barrett::systems::connect(jvFilter.output, velsat.input);

  barrett::systems::forceConnect(ball_popping::message.output, nh.input);
  barrett::systems::connect(nh.output, shb.ball_input);
  barrett::systems::connect(wam.jpOutput, gc.input);

  barrett::systems::connect(wam.toolPosition.output, modcp.input);
  barrett::systems::connect(modcp.output, shb.input);

  barrett::systems::connect(shb.output, modforce.input);
  barrett::systems::connect(modforce.output, cf_tf2jt.input);

  barrett::systems::connect(wam.toolVelocity.output, invvel.input);
  barrett::systems::connect(invvel.output, fof.input);

  barrett::systems::connect(wam.kinematicsBase.kinOutput, cf_tf2jt.kinInput); 
  barrett::systems::connect(fof.output, shb.wamVel_input);
  barrett::systems::connect(cf_tf2jt.output, jtSum.getInput(0));
  barrett::systems::connect(gc.output, jtSum.getInput(1));
  barrett::systems::connect(velsat.output, jtSum.getInput(2));
  barrett::systems::connect(jtSum.output, jtsat.input);

  //Modify safety limits
  safety_module->setTorqueLimit(4);
  safety_module->setVelocityLimit(2);
  barrett::systems::connect(jtsat.output, wam.input);
  while (true) {

    wam.idle();
    barrett::btsleep(0.02);
    while (true) {
      safety_module->getPendantState(&pendant_state);
      // Adding this will cause the new position generation to delay thereby making the program to wait for a new ball
      // However, without the delay the pendant state will not be updated, hence the program will not quit on Estop
      // barrett::btsleep(0.02);
      // Generates random joint angles within the joint range and does the forward kinematics for them
      for (size_t j = 0; j < DOF; j++) {
        rand_ang[j] = (kJointMax[j] - kJointMin[j]) * ( (double)rand() / (double)RAND_MAX ) + kJointMin[j];
      }
      shb.cart_pos = FwKin.findFwKinPosition(rand_ang);
      shb.cart_pos[0] = - shb.cart_pos[0];
      shb.cart_pos[1] = - shb.cart_pos[1];
      shb.cart_pos[0] = shb.cart_pos[0] + 1;
      if (shb.inEllipsoid(shb.cart_pos) && shb.isDistanced(shb.cart_pos) && (shb.cart_pos[2] >= (shb.getUserMin())[2] + shb.BALLR)){
        shb.new_pos = true;
      }
      while(shb.new_pos && !ball_popping::game_exit){
        barrett::btsleep(0.02);
      }
      shb.cart_pos.setZero();
      rand_ang.setZero();
      if (ball_popping::game_exit || pendant_state.pressedButton == barrett::SafetyModule::PendantState::ESTOP){ 
        product_manager.getPuck(1)->setProperty(product_manager.getPuck(1)->getBus(), 1, 8, 3);
        product_manager.getPuck(2)->setProperty(product_manager.getPuck(2)->getBus(), 2, 8, 3);
        product_manager.getPuck(3)->setProperty(product_manager.getPuck(3)->getBus(), 3, 8, 3);
        barrett::systems::disconnect(wam.input);
        wam.idle();
        return 0;
      }
    }
  }
  return 0;
}
