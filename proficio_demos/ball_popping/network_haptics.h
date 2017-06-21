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

/** @file network_haptics.h
 *
 *  Handles communication between the C++ program and the python visualization.
 *
 *  I/O    | Description
 *  ------ | ------------
 *  input  | Cartesian position of the robot joints
 *  output | Cartesian force for user gravity compensation
 *
 *  @param execution_manager    A pointer to the Wam execution manager
 *  @param remote_host          A character array containing the IP address of the
 *                              computer running the visualization
 *  @param port_src             Communication port for the C++ program, default
 *                              5557
 *  @param port_dest            Communication port for the python
 *                              visualization, default 5556
 *  @param sys_name             Name of the system
 *
 *  NOTICE: This program is for demonstration purposes only. It is not approved
 *  for clinical use.
 */


#ifndef PROFICIO_DEMOS_BALL_POPPING_NETWORK_HAPTICS_H_
#define PROFICIO_DEMOS_BALL_POPPING_NETWORK_HAPTICS_H_

#include <arpa/inet.h>   // For inet_pton()
#include <fcntl.h>       // To change socket to nonblocking mode
#include <sys/socket.h>  // For sockets
#include <unistd.h>      // for close()

#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/tuple/tuple.hpp>

#include <barrett/detail/ca_macro.h>                        // NOLINT(build/include_order)
#include <barrett/os.h>                                     // NOLINT(build/include_order)
#include <barrett/systems/abstract/single_io.h>             // NOLINT(build/include_order)
#include <barrett/thread/disable_secondary_mode_warning.h>  // NOLINT(build/include_order)
#include <barrett/units.h>                                  // NOLINT(build/include_order)

#include <proficio/tools/forward_kinematics.h>              // NOLINT(build/include_order)

// @TODO(ab): Template this to take different input and output data types, so
// the same class can be used for different demos.
class NetworkHaptics
    : public barrett::systems::SingleIO<
        barrett::math::Vector<15>::type,
        boost::tuple<double, double, double, double, double, double, double> > {
  BARRETT_UNITS_TYPEDEFS(15);  // defines v_type to have length 15

 public:
  static const int kSizeOfMessageSend = 15 * sizeof(double);
  static const int kSizeOfMessageRecv = 7 * sizeof(double);

  /** Set up networking:
   *  - Create socket
   *  - Set socket to non-blocking and set flags
   *  - Set up buffer size
   *  - Set up the bind address
   *  - Set up address of remote host
   *  - Call "connect" to set datagram destination
   */
  explicit NetworkHaptics(barrett::systems::ExecutionManager* execution_manager,
                          const char* remote_host, int port_src = 5557,
                          int port_dest = 5556,
                          const std::string& sys_name = "NetworkHaptics")
    : barrett::systems::SingleIO<v_type, boost::tuple<
        double, double, double, double, double, double, double> > (sys_name),
      this_socket_(-1), input_data_(0.0), output_data_(0.0) {
    int status;
    long flags;
    int buflen;
    unsigned int buflenlen;
    struct sockaddr_in bind_addr;
    struct sockaddr_in their_addr;

    // @TODO(ab): Restructure to detect problems without throwing a runtime
    // error and quit gracefully. Maybe use an init() function instead of doing
    // this in the constructor?

    // Create socket
    this_socket_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (this_socket_ == -1) {
      ctor_error_handler("Could not create socket", __func__);
    }

    // Set socket to non-blocking, set flag associated with open file
    flags = fcntl(this_socket_, F_GETFL, 0);
    if (flags < 0) {
      ctor_error_handler("Could not get socket flags.", __func__);
    }
    flags |= O_NONBLOCK;
    status = fcntl(this_socket_, F_SETFL, flags);
    if (status < 0) {
      ctor_error_handler("Could not set socket flags.", __func__);
    }

    // TODO(dc): Maybe set UDP buffer size?
    buflenlen = sizeof(buflen);
    status = getsockopt(this_socket_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen,
                        &buflenlen);
    if (status) {
      ctor_error_handler("Could not get output buffer size.", __func__);
    }
    barrett::logMessage("%s: Note, output buffer is %d bytes.")
        % __func__ % buflen;

    buflenlen = sizeof(buflen);
    buflen = 5 * kSizeOfMessageRecv;
    status = setsockopt(this_socket_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen,
                        buflenlen);
    if (status) {
      (barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Ctor failed  %s: Could not set "
          "output buffer size.") % __func__).raise<std::runtime_error>();
    }

    buflenlen = sizeof(buflen);
    status = getsockopt(this_socket_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen,
                        &buflenlen);
    if (status) {
      ctor_error_handler("Could not set output buffer size.", __func__);
    }
    barrett::logMessage("%s: Note, output buffer is %d bytes.")
        % __func__ % buflen;

    // Set up the bind address
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(port_src);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    status = bind(this_socket_, (struct sockaddr *)&bind_addr,
                  sizeof(bind_addr));
    if (status == -1) {
      barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Constructor failed %s: "
          "Could not bind to socket on port %d") %
          __func__ % port_src;
      throw std::runtime_error(
          "(NetworkHaptics::NetworkHaptics): "
          "Could not bind to socket on port.");
    }

    // Set up the other guy's address
    their_addr.sin_family = AF_INET;
    their_addr.sin_port = htons(port_dest);
    status = !inet_pton(AF_INET, remote_host, &their_addr.sin_addr);
    if (status) {
      barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Constructor failed %s: "
          "Bad IP argument '%s'.") %
          __func__ % remote_host;
      throw std::runtime_error(
          "(NetworkHaptics::NetworkHaptics): Bad IP argument.");
    }

    // Call "connect" to set datagram destination
    status = connect(this_socket_, (struct sockaddr *)&their_addr,
                     sizeof(struct sockaddr));
    if (status) {
      ctor_error_handler("Could not set datagram destination.", __func__);
    }

    // Explicitly tell the execution manager to run this system every timestep.
    // This way it will run even if the output is not connected to anything.
    if (execution_manager != NULL) {
      execution_manager->startManaging(*this);
    }
  }

  virtual ~NetworkHaptics() {
    mandatoryCleanUp();
    close(this_socket_);
  }

  /** Handles basic errors in the constructor by logging them with
   *  barrett::logMessage, and throwing a runtime error
   *
   *  @param failure_type   brief description of failure
   *  @param func_name      name of the function where the failure occured
   */
  void ctor_error_handler(const std::string& failure_type,
                          const std::string& func_name) {
    std::string log_msg =
        "(NetworkHaptics::NetworkHaptics): Constructor failed ";
    log_msg.append(func_name);
    log_msg.append(failure_type);
    std::string error_msg = "(NetworkHaptics::NetworkHaptics): ";
    error_msg.append(failure_type);
    barrett::logMessage(log_msg);
    throw std::runtime_error(error_msg);
  }

 protected:
  int this_socket_;
  v_type input_data_;
  boost::tuple<double, double, double, double,        // NOLINT
               double, double, double> output_data_;

  /** Send/receive data to/from the python visualization. */
  virtual void operate() {
    input_data_ = input.getValue();

    // send() and recv() cause switches to secondary mode. The socket is
    // non-blocking, so this *probably* won't impact the control-loop
    // timing that much...
    // @TODO(ab): See about making this real-time safe.

    barrett::thread::DisableSecondaryModeWarning dsmw;

    send(this_socket_, input_data_.data(), kSizeOfMessageSend, 0);
    recv(this_socket_, &output_data_, kSizeOfMessageRecv, 0);

    outputValue->setData(&output_data_);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(NetworkHaptics);
};

#endif  // PROFICIO_DEMOS_BALL_POPPING_NETWORK_HAPTICS_H_
