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

/** @file cube_sphere.h
 *
 *  Network haptics system
 *
 *  I/O    | Description
 *  ------ | ------------
 *  input  | Cartesian position of the robot joints
 *  output | Cartesian force for user gravity compensation
 *
 *  @param execution_manager
 *  @param remote_host
 *  @param grav_comp
 *  @param port_src
 *  @param port_dest
 *  @param sys_name
 *
 *  NOTICE: This program is for demonstration purposes only. It is not approved
 *  for clinical use.
 */

#ifndef PROFICIO_DEMOS_CUBE_SPHERE_CUBE_SPHERE_H_
#define PROFICIO_DEMOS_CUBE_SPHERE_CUBE_SPHERE_H_

#include <arpa/inet.h>  /* For inet_pton() */
#include <fcntl.h>      /* To change socket to nonblocking mode */
#include <sys/socket.h> /* For sockets */
#include <unistd.h>     /* for close() */

#include <stdexcept>
#include <string>

#include <boost/tuple/tuple.hpp>

#include <barrett/detail/ca_macro.h>                        // NOLINT(build/include_order)
#include <barrett/os.h>                                     // NOLINT(build/include_order)
#include <barrett/systems/abstract/single_io.h>             // NOLINT(build/include_order)
#include <barrett/thread/disable_secondary_mode_warning.h>  // NOLINT(build/include_order)
#include <barrett/units.h>                                  // NOLINT(build/include_order)

#include <proficio/systems/utilities.h>                     // NOLINT(build/include_order)

template <size_t DOF>
class NetworkHaptics
    : public barrett::systems::SingleIO<
          barrett::units::CartesianPosition::type,
          boost::tuple<double, barrett::units::CartesianForce::type> > {
  BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

 public:
  static const int SIZE_OF_MSG = 3 * sizeof(double);
  static const int SIZE_OF_MSG_RECV = 4 * sizeof(double);

  /** Set up networking:
   *  - Create socket
   *  - Set socket to non-blocking and set flags
   *  - Set up buffer size
   *  - Set up the bind address
   *  - Set up address of remote host
   *  - Call "connect" to set datagram destination
   */
  explicit NetworkHaptics(
      barrett::systems::ExecutionManager* execution_manager, const char* remote_host,
      proficio::systems::UserGravityCompensation<DOF>* grav_comp,
      int port_src = 5557, int port_dest = 5556,
      const std::string& sys_name = "NetworkHaptics")
      : barrett::systems::SingleIO<cp_type, boost::tuple<double, cf_type> >(
            sys_name),
        sock_(-1),
        curr_pos_(0.0),
        user_grav_comp(grav_comp) {
    int err;
    long flags;  // NOLINT(runtime/int)
    int buflen;
    unsigned int buflenlen;
    struct sockaddr_in bind_addr;
    struct sockaddr_in their_addr;
    cf_type cf_zero;
    cf_zero.setZero();
    tuple_msg.get<1>() = cf_zero;
    tuple_msg.get<0>() = 10;

    /* Create socket */
    sock_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock_ == -1) {
      ctor_error_handler("Could not create socket", __func__);
    }

    /* Set socket to non-blocking, set flag associated with open file */
    flags = fcntl(sock_, F_GETFL, 0);
    if (flags < 0) {
      ctor_error_handler("Could not get socket flags.", __func__);
    }

    flags |= O_NONBLOCK;
    err = fcntl(sock_, F_SETFL, flags);
    if (err < 0) {
      ctor_error_handler("Could not set socket flags.", __func__);
    }

    /* Maybe set UDP buffer size? */
    buflenlen = sizeof(buflen);
    err = getsockopt(sock_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen, &buflenlen);  // NOLINT
    if (err) {
      ctor_error_handler("Could not get output buffer size.", __func__);
    }
    barrett::logMessage("%s: Note, output buffer is %d bytes.") % __func__ %
        buflen;

    buflenlen = sizeof(buflen);
    buflen = 5 * SIZE_OF_MSG_RECV;
    err = setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, (char*)&buflen, buflenlen);  // NOLINT
    if (err) {
      ctor_error_handler("Could not set output buffer size.", __func__);
    }

    /* Set up the bind address */
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(port_src);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    err = bind(sock_, (struct sockaddr*)&bind_addr, sizeof(bind_addr));
    if (err == -1) {
      barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Constructor failed %s: "
          "Could not bind to socket on port %d") %
          __func__ % port_src;
      throw std::runtime_error(
          "(NetworkHaptics::NetworkHaptics): "
          "Could not bind to socket on port.");
    }

    /* Set up the other guy's address */
    their_addr.sin_family = AF_INET;
    their_addr.sin_port = htons(port_dest);
    err = !inet_pton(AF_INET, remote_host, &their_addr.sin_addr);
    if (err) {
      barrett::logMessage(
          "(NetworkHaptics::NetworkHaptics): Constructor failed %s: "
          "Bad IP argument '%s'.") %
          __func__ % remote_host;
      throw std::runtime_error(
          "(NetworkHaptics::NetworkHaptics): Bad IP argument.");
    }

    /* Call "connect" to set datagram destination */
    err =
        connect(sock_, (struct sockaddr*)&their_addr, sizeof(struct sockaddr));
    if (err) {
      ctor_error_handler("Could not set datagram destination.", __func__);
    }

    if (execution_manager != NULL) {
      execution_manager->startManaging(*this);
    }
  }

  virtual ~NetworkHaptics() {
    mandatoryCleanUp();
    close(sock_);
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
  int sock_;
  cp_type curr_pos_;
  boost::tuple<double, cf_type> tuple_msg;  // NOLINT
  proficio::systems::UserGravityCompensation<DOF>* user_grav_comp;

  /** Send current position. Get user gravity compensation message
   *  and set/increment/decrement the gain as specified
   */
  virtual void operate() {
    curr_pos_ = input.getValue();
    // send() and recv() cause switches to secondary mode. The socket is
    // non-blocking, so this *probably* won't impact the control-loop
    // timing that much...
    barrett::thread::DisableSecondaryModeWarning dsmw;

    send(sock_, curr_pos_.data(), SIZE_OF_MSG, 0);
    recv(sock_, &tuple_msg, SIZE_OF_MSG_RECV, 0);
    if (tuple_msg.get<0>() == 0)
      user_grav_comp->setGainZero();
    else if (tuple_msg.get<0>() == 1)
      user_grav_comp->incrementGain();
    else if (tuple_msg.get<0>() == 2)
      user_grav_comp->decrementGain();

    outputValue->setData(&tuple_msg);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(NetworkHaptics);
};

#endif  // PROFICIO_DEMOS_CUBE_SPHERE_CUBE_SPHERE_H_
