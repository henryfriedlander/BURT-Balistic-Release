/*
 * Copyright 2016 Barrett Technology <support@barrett.com>
 *
 * This file is part of proficio_toolbox.
 *
 * This version of proficio_toolbox is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This version of proficio_toolbox is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General 
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this version of proficio_toolbox. If not, see <http://www.gnu.org/licenses/>.
 */

/** @file standard_proficio_main.h
 *
 * Defines a boilerplate main() function that initializes a Proficio.
 *
 * This "standard" main() function simply initializes the Proficio in the normal
 * way, by:
 *  - Waiting for the Proficio to be Shift-idled, if it's not already idled
 *  - Waking all Pucks found on the bus
 *  - Zeroing the Proficio, if it's not already zeroed
 *  - Waiting for the Proficio to be Shift-activated if the Control Pendant is
 *    to be used
 *  - Calling the proficio_main() function that the user is responsible for
 *    defining
 *  - Bypassing the waiting sequences that require the control pendant's input
 *    if the NO_CONTROL_PENDANT macro is defined
 *
 * The behavior is appropriate for many applications, but there is no issue with
 * writing a custom main() function.
 *
 * The proficio_main() function must be able to accept a reference to a 3-DOF
 * barrett::systems::Wam.
 */

#ifndef PROFICIO_STANDARD_PROFICIO_MAIN_H_
#define PROFICIO_STANDARD_PROFICIO_MAIN_H_

#include <barrett/exception.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/systems.h>

#include <proficio/tools/configure_safety_module.h>

enum Config {
  LEFT,
  RIGHT
};

// In the following macros, SMF stands for "standard main function"

/** \def NO_CONTROL_PENDANT
 * The programmer should define this macro before including
 * standard_proficio_main.h in order to activate the robot without the control
 * pendant.
 */
#ifdef NO_CONTROL_PENDANT
#  define BARRETT_SMF_CONFIGURE_PM
#  define BARRETT_SMF_DONT_PROMPT_ON_ZEROING
#  define BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#endif

/** \def BARRETT_SMF_VALIDATE_ARGS
 * The programmer should define this macro before including
 * standard_proficio_main.h if the program requires command line arguments. In
 * this case, the programmer must define the validate_args() function in their
 * code, and this function will be called before proficio_main() begins.
 */
#ifdef BARRETT_SMF_VALIDATE_ARGS
bool validate_args(int argc, char** argv);
#endif

#ifdef BARRETT_SMF_CONFIGURE_PM
namespace proficio {
/** Sets up the safety module for pendantless operation and ensures that it is
 * in the correct state to start the robot.
 */
bool configurePm(int argc, char** argv, ::barrett::ProductManager& pm) {
  ::barrett::SafetyModule* sm = pm.getSafetyModule();
  proficio::configureSafetyModule(sm);
  ::barrett::SafetyModule::PendantState ps;

  sm->getPendantState(&ps);
  if (ps.pressedButton == ::barrett::SafetyModule::PendantState::ESTOP) {
    std::cout << "Please release the Proficio ESTOP to start." << std::endl;
  }
  while (ps.pressedButton == ::barrett::SafetyModule::PendantState::ESTOP) {
    sm->getPendantState(&ps);
    usleep(100000);  // time is in microseconds
  }
  if (sm->getMode() == ::barrett::SafetyModule::ESTOP ||
      sm->getMode() == ::barrett::SafetyModule::IDLE) {
    // Even if the SafetyModule is already in IDLE mode, set it to IDLE again
    // because this tells the SafetyModule to send some commands to the motor
    // pucks.
    sm->setMode(::barrett::SafetyModule::IDLE);
    // Second argument is false to suppress printing the "Please shift-idle
    // the robot" message.
    sm->waitForMode(::barrett::SafetyModule::IDLE, false);
  }
  return true;
}
}  // namespace proficio
#endif

#ifndef BARRETT_SMF_NO_DECLARE
/** Custom main function to be defined by the user. This function is called
 * at the end of the main function defined here.
 */
template<size_t DOF>
int proficio_main(int argc, char** argv, ::barrett::ProductManager& pm,
                  ::barrett::systems::Wam<DOF>& wam, const Config& side);
#endif

#ifndef BARRETT_SMF_DONT_WAIT_FOR_SHIFT_ACTIVATE
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE true
#else
#  define BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE false
#endif

#ifndef BARRETT_SMF_DONT_PROMPT_ON_ZEROING
#  define BARRETT_SMF_PROMPT_ON_ZEROING true
#else
#  define BARRETT_SMF_PROMPT_ON_ZEROING false
#endif

#ifndef BARRETT_SMF_WAM_CONFIG_PATH
#  define BARRETT_SMF_WAM_CONFIG_PATH NULL
#endif

/** The main function configures the product manager and the robot for
 * operation with or without the pendant, depending on whether
 * NO_CONTROL_PENDANT is defined. Then it calls proficio_main(), which is
 * defined by the programmer in their application. If BARRETT_SMF_VALIDATE_ARGS
 * is defined, it calls the validate_args() function, which is also defined by
 * the programmer, before configuring the robot. validate_args() is intended to
 * validate command-line arguments passed to the program.
 *
 * Returns 2 if validate_args() returns false or if product manager
 * configuration fails. Returns 0 if no Proficio was found or if a run-time
 * error is caught (which is interpreted as failure to instantiate the
 * connection). Otherwise, returns the return value of the proficio_main()
 * function.
 */
// TODO(ab): Update return values so 0 is a success. Requires corresponding
// changes in other proficio_toolbox programs (homecheck and state_toggle,
// at least) and in proficio_gui (scripts/wxProficio_gui.py).
int main(int argc, char** argv) {
  // Give us pretty stack-traces when things die
  ::barrett::installExceptionHandler();

  try {
#ifdef BARRETT_SMF_VALIDATE_ARGS
    if (!validate_args(argc, argv)) {
      return 2;
    }
#endif

    ::barrett::ProductManager pm;
#ifdef BARRETT_SMF_CONFIGURE_PM
    if (!proficio::configurePm(argc, argv, pm)) {
      return 2;
    }
#endif

#ifndef NO_CONTROL_PENDANT
    pm.waitForWam(BARRETT_SMF_PROMPT_ON_ZEROING);
#endif
    pm.wakeAllPucks();

    // ProductManager::foundWam3() checks for a robot with three motor pucks.
    // Currently, this is the method we use to check whether a Proficio is
    // connected. This function is likely to be replaced in the future with a
    // Proficio-specific function.
    if (pm.foundWam3()) {
      ::barrett::systems::Wam<3>* wam = pm.getWam3(
          BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE, BARRETT_SMF_WAM_CONFIG_PATH);
      BARRETT_UNITS_TEMPLATE_TYPEDEFS(3);
#ifdef NO_CONTROL_PENDANT
      ::barrett::SafetyModule* sm = pm.getSafetyModule();
      if (sm->getMode() != ::barrett::SafetyModule::ACTIVE) {
        // Set joint torque command to zero before activating
        ::barrett::systems::ExposedOutput<jt_type> zeroTorque;
        jt_type zero(0.0);
        zeroTorque.setValue(zero);
        ::barrett::systems::connect(zeroTorque.output, wam->input);
        usleep(50000);  // sleep time in us
        // Activate the safety module
        sm->setMode(::barrett::SafetyModule::ACTIVE);
        sm->waitForMode(::barrett::SafetyModule::ACTIVE, false);
        ::barrett::systems::disconnect(wam->input);
      }
      wam->getLowLevelWam().getPuckGroup().setProperty(
          ::barrett::Puck::MODE, ::barrett::MotorPuck::MODE_TORQUE);
#endif
      // Check for left or right configuration.
      // TODO(ab): This section will change once hardware support for
      // configuration detection is available.
      jp_type jpos = wam->getJointPositions();
      Config side;
      if (jpos[2] < 0) { side = LEFT;
      } else if (jpos[2] > 0) { side = RIGHT; }

      return proficio_main(argc, argv, pm,
                           *pm.getWam3(BARRETT_SMF_WAIT_FOR_SHIFT_ACTIVATE,
                                       BARRETT_SMF_WAM_CONFIG_PATH),
                           side);
    } else {
      std::cout << "ERROR: No Proficio was found. Perhaps you have found a bug"
        << " in ProductManager::waitForWam()." << std::endl;
      return 0;
    }
  } catch (std::runtime_error& e) {
    std::cout << "ERROR: Could not instantiate the connection. Please check"
      << " the logs."
      << std::endl;
    return 0;
  }
}

#endif  // PROFICIO_STANDARD_PROFICIO_MAIN_H_
