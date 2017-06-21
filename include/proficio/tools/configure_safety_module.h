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

/** @file configure_safety_module.h
 *
 * A function to configure the safety module on board the robot for pendantless
 * operation. This allows the robot to start up without the user needing to
 * touch the pendant, once the e-stop is disengaged.
 *
 */

/************************** IMPORTANT SAFETY NOTICE ****************************
 * The function below enables remote activation of Barrett's Proficio robot. It
 * is designed to support the use of the Proficio in a clinical environment with
 * firmware and software specifically designed to ensure safety of users. It
 * allows the robot to be turned on and activated from software.
 *
 * IT IS SUPPOSED TO BE ONLY USED IN THE CLINICAL VERSIONS OF OUR ROBOT.
 *
 * This code allows the robot to become active without any direct human
 * intervention. This behavior violates recommendations from the Robotics
 * Industries Association. Please think carefully before using this code in your
 * application. Contact support@barrett.com with any questions.
 *
 * This functionality requires SafetyModule firmware version >= 197.
 ************************** IMPORTANT SAFETY NOTICE ***************************/


#ifndef PROFICIO_TOOLS_CONFIGURE_SAFETY_MODULE_H_
#define PROFICIO_TOOLS_CONFIGURE_SAFETY_MODULE_H_

#include <barrett/products/puck.h>
#include <barrett/products/safety_module.h>

namespace proficio {
inline void configureSafetyModule(const barrett::SafetyModule* safety_module) {
  using barrett::Puck;

  Puck& safety_puck = *safety_module->getPuck();
  // internal tools - ask BZ
  safety_puck.setProperty(Puck::LOCK, 1234, true);
  safety_puck.setProperty(Puck::LOCK, 18384, true);
  safety_puck.setProperty(Puck::LOCK, 23, true);
  safety_puck.setProperty(Puck::LOCK, 3145, true);
  safety_puck.setProperty(Puck::LOCK, 1024, true);
  safety_puck.setProperty(Puck::LOCK, 1, true);
}
}  // namespace proficio

#endif  // PROFICIO_TOOLS_CONFIGURE_SAFETY_MODULE_H_
