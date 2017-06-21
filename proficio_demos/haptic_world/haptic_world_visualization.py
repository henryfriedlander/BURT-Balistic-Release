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

'''
@file haptic_world_vizualization.py

THIS CODE IS NOT APPROVED FOR CLINICAL USE
'''

import sys
import time
import math
import select
import struct
import numpy
import Image
from socket import socket, AF_INET, SOCK_DGRAM
from visual import *


def get_remote_host():
    if len(sys.argv) == 2:
        return sys.argv[1]
    else:
        print("Defaulting to 127.0.0.1 for WAM IP Address")
        return "127.0.0.1"


def make_socket(remoteHost):
    port_src = 5556
    port_dest = 5557
    sock = socket(AF_INET, SOCK_DGRAM)
    sock.bind(('', port_src))
    sock.connect((remoteHost, port_dest))
    return sock


def set_up_display(yMiddle, zMiddle):
    scene = display()
    scene.fullscreen = True
    scene.center = (0.0, yMiddle, zMiddle + 0.15)  # 0.4
    scene.fov = math.pi / 2.0
    scene.scale = (1.3, 1.3, 1.3)
    scene.forward = (-0.00250513, 0.517794, -0.855502)
    return scene


def main():
    remote_host = get_remote_host()
    # messages contain 10 doubles
    #    3 Proxy Position
    #    3 Proxy Velocity
    #    3 Ball Position
    #    1 Button Press
    msg_format = "d" * 10
    msg_format_send = "d" * 11
    msg_size = struct.calcsize(msg_format)
    sock = make_socket(remote_host)

    # Visuals
    xMin, xMax = 0.3, 0.7
    yMin, yMax = -0.25, 0.25
    zMin, zMax = 0.0, 0.5

    xSize = xMax - xMin
    ySize = yMax - yMin
    zSize = zMax - zMin

    xMiddle = xMin + (xSize / 2)
    yMiddle = yMin + (ySize / 2)
    zMiddle = zMin + (zSize / 2)

    proxyPos = (xMiddle, yMiddle, zMiddle)
    proxySize = 0.03
    proxyColor = (0.0, 1.0, 0.0)
    proxyVel = (0.0, 0.0, 0.0)

    ballPos = (0.35, 0.2, 0.75)
    ballSize = 0.075

    buttonPress = False

    scene = set_up_display(yMiddle, zMiddle)

    f = frame()
    f.rotate(angle=-pi / 2.0, axis=(1, 0, 0), origin=(0, 0, 0))
    f.rotate(angle=-pi / 2.0, axis=(0, 1, 0), origin=(0, 0, 0))
    f.rotate(angle=pi * 0.2, axis=(1, 0, 0), origin=(0, 0, 0))

    wall_size = 0.002

    corduroy_name = "textures/corduroy"
    im_corduroy = Image.open(corduroy_name + ".png")
    materials.saveTGA(corduroy_name, im_corduroy)

    barrett_name = "textures/barrett_logo"
    im_barrett = Image.open(barrett_name + ".png")
    im_barrett.resize((1024, 256), Image.ANTIALIAS)
    materials.saveTGA(barrett_name, im_barrett)

    magnet_name = "textures/magnet"
    im_magnet = Image.open(magnet_name + ".png")
    materials.saveTGA(magnet_name, im_magnet)

    corduroy = materials.texture(data=materials.loadTGA(corduroy_name),
                                 mapping="rectangular")
    barrett = materials.texture(data=materials.loadTGA(barrett_name),
                                 mapping="rectangular")
    magnet = materials.texture(data=materials.loadTGA(magnet_name),
                                 mapping="rectangular")

    custom_orange = (1.0, 0.7, 0.2)
    custom_purple = (228.0 / 255.0, 40.0 / 255.0, 184.0 / 255.0)
    barrett_blue = (47.0 / 255.0, 40.0 / 255.0, 228.0 / 255.0)
    barrett_brown = (117.0 / 255.0, 69.0 / 255.0, 27.0 / 255.0)
    custom_light_blue = (5.0 / 255.0, 207.0 / 255.0, 255.0 / 255.0)

    floor = box(pos=(xMiddle, yMiddle, zMin),
                length=xSize, height=ySize, width=wall_size,
                frame=f, material=corduroy)
    barrett_wall = box(pos=(xMin + 0.01, yMiddle, zMiddle),
                       length=wall_size, height=ySize - 0.1, width=zSize / 4.8,
                       frame=f, material=barrett)
    barrett_wall.rotate = (pi / 4, (1, 0, 0), barrett_wall.pos)
    back_wall = box(pos=(xMin, yMiddle, zMiddle),
                    length=wall_size, height=ySize, width=zSize,
                    frame=f, color=barrett_blue)
    left_wall = box(pos=(xMiddle, yMin, zMiddle),
                    length=xSize, height=wall_size, width=zSize,
                    frame=f, color=color.yellow)
    right_wall = box(pos=(xMiddle, yMax, zMiddle),
                     length=xSize, height=wall_size, width=zSize,
                     frame=f, color=custom_orange)
    magnet_wall = cylinder(pos=(xMiddle, yMax, zMiddle),
                           axis=(0, -0.0012, 0), radius=0.15,
                           frame=f, material=magnet)
    ceiling = box(pos=(xMiddle, yMiddle, zMax),
                  length=xSize, height=ySize, width=wall_size,
                  frame=f, color=custom_purple)
    left_button = cylinder(pos=(xMiddle, yMin, zMiddle),
                           axis=(0.0, 0.01, 0.0), radius=0.15,
                           frame=f, color=color.red)
    ball = sphere(pos=(ballPos[0:3]), radius=ballSize,
                  material=materials.earth, frame=f)
    # proxy for robot endpoint position
    proxy = sphere(pos=(proxyPos[0:3]), radius=proxySize,
                   color=color.green, frame=f)

    run_game = True
    while run_game:
        msg = struct.unpack(msg_format, sock.recv(msg_size))
        proxy.pos = (msg[0], msg[1], msg[2])
        proxyVel = (msg[3:6])
        ball.pos = (msg[6], msg[7], msg[8])
        buttonPress = (bool)(msg[9])
        if buttonPress == True:
            left_button.axis = (0.0, 0.004, 0.0)
            left_wall.color = (color.green)
        else:
            left_button.axis = (0.0, 0.01, 0.0)
            left_wall.color = (color.yellow)
        if  abs((proxy.pos[0] - proxySize) - back_wall.pos[0]) < 0.01:
            back_wall.color = barrett_brown
        else:
            back_wall.color = barrett_blue

        if abs((proxy.pos[2] + proxySize) - ceiling.pos[2]) < 0.01:
            ceiling.color = custom_light_blue
        else:
            ceiling.color = custom_purple
        if scene.kb.keys:  # If any input was pressed
            s = scene.kb.getkey()
            if s == 'up':
                gcomp = 1
            elif s == 'down':
                gcomp = 2
            elif s == 'delete':
                gcomp = 0
            else:  # Any other key, do nothing
                gcomp = 100
            sock.send(struct.pack(msg_format_send, gcomp,
                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1))
            sock.send(struct.pack(msg_format_send, 100,
                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1))
    sock.close()

if __name__ == "__main__":
    main()
