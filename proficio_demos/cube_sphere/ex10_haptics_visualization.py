#!/usr/bin/python
#  Copyright 2016 Barrett Technology support@barrett.com
#
#  This file is part of proficio_toolbox.
#
#  This version of proficio_toolbox is free software: you can redistribute it
#  and/or modify it under the terms of the GNU General Public License as
#  published by the Free Software Foundation, either version 3 of the
#  License, or (at your option) any later version.
#
#  This version of proficio_toolbox is distributed in the hope that it will be
#  useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this version of proficio_toolbox.  If not, see
#  http://www.gnu.org/licenses/.

"""
@file ex10_haptics_visualization.py

Visualization for cube_sphere demo

NOTICE: This program is for demonstration purposes only. It is not approved
for clinical use.THIS CODE IS NOT APPROVED FOR CLINICAL USE
"""

import sys
from socket import socket, AF_INET, SOCK_DGRAM
from time import sleep
import struct
from visual import *
import numpy


def get_remote_host():
    """ Returns IP address from command line argument. 
        Defaults to 127.0.0.1 """
    if len(sys.argv) == 2:
        return sys.argv[1]
    else:
        print("Defaulting to 127.0.0.1 for WAM IP Address")
        return "127.0.0.1"


def make_socket(remote_host):
    """ Create and return a socket connected to remote_host
    @param remote_host      IP address """
    port_src = 5556
    port_dest = 5557
    sock = socket(AF_INET, SOCK_DGRAM)
    sock.bind(('', port_src))
    sock.connect((remote_host, port_dest))
    return sock


def main():
    """ Creates a visual world with a cube and sphere.

    Uses a socket to send user gravity compensation settings and receive
    robot endpoint position. """
    remote_host = get_remote_host()
    msg_format = "d" * 3  # messages contain 3 doubles
    msg_format_send = "d" * 4  # messages contain 4 doubles
    msg_size_send = struct.calcsize(msg_format_send)
    msg_size = struct.calcsize(msg_format)
    sock = make_socket(remote_host)

    # set up visual
    scene.fullscreen = False
    #scene.cursor.visible = False
    #scene.autocenter = True
    #scene.autoscale = True
    scene.center = (0, -0.4, 1)
    scene.scale = (2.5, 2.5, 2.5)

    f = frame()
    f.rotate(angle=-pi / 2.0, axis=(1, 0, 0), origin=(0, 0, 0))
    f.rotate(angle=-pi / 2.0, axis=(0, 1, 0), origin=(0, 0, 0))
    f.rotate(angle=pi * 0.2, axis=(1, 0, 0), origin=(0, 0, 0))

    # unit vectors
    arrow(pos = (0,0,0), axis = (-0.3,0,0), color = color.red, frame = f)
    #arrow(pos = (0,0,0), axis = (0,0.3,0), color = color.green, frame = f)
    #arrow(pos = (0,0,0), axis = (0,0,0.3), color = color.blue, frame = f)

    # floor
    xMin, xMax = -1.0, 0.4
    yMin, yMax = -0.8, 0.8
    z = -0.5
    step = 0.1
	
	# draws grid on the screen
    for x in numpy.linspace(xMin, xMax, (xMax - xMin) / step):
        curve(pos=[(x, yMin, z), (x, yMax, z)], frame=f)
    for y in numpy.linspace(yMin, yMax, (yMax - yMin) / step):
        curve(pos=[(xMin, y, z), (xMax, y, z)], frame=f)

    # haptic objects
    #sphere(pos=(0.4, -0.15, 0), radius=0.02, color=color.blue, opacity=0.6,
    #       frame=f)
    sphere(pos=(0.4, -0.15, 0.15), radius=0.01, color=color.green, opacity=0.6,
           frame=f)
    #box(pos=(0.35, 0.2, 0), length=0.19, height=0.19, width=0.19,
    #    color=color.yellow, opacity=0.6, frame=f)

    # end point
    ep = sphere(pos=(0, 0, 0), radius=0.01, color=color.red, frame=f)

    grav_comp = 100
    
    posTargetPos = [(0.4, -0.15, 0.15), (0.4, -0.15, -0.15), (0.4, 0, 0), (0.4, -0.3, 0), (0.55, -0.15, 0), (0.25, -0.15, 0)]
    while True:
        if scene.kb.keys:  # If any input was pressed
            s = scene.kb.getkey()
            if s == 'up':
                grav_comp = 1  # increase user gravity compensation
            elif s == 'down':
                grav_comp = 2  # decrease user gravity compensation
            elif s == 'delete':
                grav_comp = 0  # reset (turn off) user gravity compensation
            else:
                grav_comp = 100  # do nothing
            sock.send(struct.pack(msg_format_send, grav_comp, 1, 1, 1))
            sock.send(struct.pack(msg_format_send, 100, 1, 1, 1))
        ep.pos = struct.unpack(msg_format, sock.recv(msg_size))
    sock.close()


if __name__ == "__main__":
    main()
