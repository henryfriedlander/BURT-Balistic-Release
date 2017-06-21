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
@file ball_popping_visualization.py

THIS CODE IS NOT APPROVED FOR CLINICAL USE
'''

import sys
from socket import socket, AF_INET, SOCK_DGRAM
from time import sleep
import struct
from visual import *
from visual.controls import *
import numpy
import random
import Image
import time
import subprocess

# args
if len(sys.argv) == 2:
    remoteHost = sys.argv[1]
else:
    print("Defaulting to 127.0.0.1 for WAM IP Address")
    remoteHost = "127.0.0.1"

# set up socket
PORT_SRC = 5556
PORT_DEST = 5557
MSG_FORMAT_SND = "ddddddd"
MSG_SIZE_SND = struct.calcsize(MSG_FORMAT_SND)
MSG_FORMAT = "ddd"
MSG_SIZE = struct.calcsize(MSG_FORMAT)
MSG_FORMAT_RECV = "ddddddddddddddd"
MSG_SIZE_RECV = struct.calcsize(MSG_FORMAT_RECV)

sock = socket(AF_INET, SOCK_DGRAM)
sock.bind(('', PORT_SRC))
#sock.settimeout(10)
sock.connect((remoteHost, PORT_DEST))
#time.sleep(5)
# floor
xMin, xMax = -1, 0.5
yMin, yMax = -1.4, 0.6
step = 0.1
zMin, zMax = 0.05, 1.55
edge_r = 0.05

# Speeds
hi_speed = 0.001
med_speed = 0.0005
low_speed = 0.00025

# force_delay
high_delay = 1.4
medium_delay = 0.7
low_delay = 0.1

# asst_force
high_force = 350
medium_force = 600
low_force = 400
zero_force = 0

# Calibration synch parameters
scale = 1
no_scale = 0
cb_started = 1
cb_not_started = 0
cb_ended = 1
cb_not_ended = 0

# gravity calibration parameters
no_act = 0
increment = 1
decrement = 2
zeroes = 3

# Game options
no_force = 0
mag_floor = 1
w_calib = 2
game = 3
game_new_session = 4
quit = 5

timeout = 10  # No of loops to wait before generating a new position

new_pos_sent = 1
old_pos_sent = 0

xSize = xMax - xMin
ySize = yMax - yMin
zSize = zMax - zMin

xMiddle = xMin + (xSize / 2)
yMiddle = yMin + (ySize / 2)
zMiddle = zMin + (zSize / 2)
wall_size = 0.01

# Setup the scene
scene.fullscreen = True
scene.center = (xMiddle - 0.15, yMiddle + 0.65, zMiddle)  # 0.4
scene.fov = math.pi / 2.5
scene.scale = (0.9, 1.3, 1.3)
scene.forward = (-0.00250513, 0.617794, -0.855502)

f = frame()
f.rotate(angle=-pi / 2.0, axis=(1, 0, 0), origin=(0, 0, 0))
f.rotate(angle=-pi / 2.0, axis=(0, 1, 0), origin=(0, 0, 0))
f.rotate(angle=pi * 0.2, axis=(1, 0, 0), origin=(0, 0, 0))

hand = "textures/HandIcon"
imC = Image.open(hand + ".png")
imC = imC.resize((512, 512), Image.ANTIALIAS)
materials.saveTGA(hand, imC)

shadow = "textures/shade"
imS = Image.open(shadow + ".png")
imS = imS.resize((400, 400), Image.ANTIALIAS)
materials.saveTGA(shadow, imS)

back_wallM = "textures/bg"
imC = Image.open(back_wallM + ".png")
imC = imC.resize((512, 512), Image.ANTIALIAS)
materials.saveTGA(back_wallM, imC)

back_ground = "textures/bg_alone"
imD = Image.open(back_ground + ".png")
imD = imD.resize((512, 512), Image.ANTIALIAS)
materials.saveTGA(back_ground, imD)

ceiling = "textures/Ceiling"
imCe = Image.open(ceiling + ".png")
imCe = imCe.resize((512, 512), Image.ANTIALIAS)
materials.saveTGA(ceiling, imCe)

h_icon = materials.texture(data=materials.loadTGA(hand),
                           mapping="rectangular")
sh_icon = materials.texture(data=materials.loadTGA(shadow),
                            mapping="rectangular")
b_wall = materials.texture(data=materials.loadTGA(back_wallM),
                           mapping="rectangular")
bg = materials.texture(data=materials.loadTGA(back_ground),
                       mapping="rectangular")
cei = materials.texture(data=materials.loadTGA(ceiling),
                        mapping="rectangular")

# haptic objects
sp = sphere(pos=(xMiddle, yMin + 0.12, zMin + wall_size + 0.12), radius=0.12,
            color=color.red, opacity=0, frame=f)
floor = box(pos=(xMiddle, yMiddle, zMin), length=xSize, height=ySize + 0.2,
            width=wall_size, color=color.gray(0.455), frame=f)
top = box(pos=(xMiddle, yMiddle, zMax), length=xSize, height=ySize + 0.2,
          width=wall_size,frame=f, material=cei)
left = box(pos=(xMiddle, yMin - 0.1, zMiddle), length=wall_size, height=xSize,
           width=zSize, interpolate=False, frame=f, material=bg)
back = box(pos=(xMin, yMiddle, zMiddle),length=wall_size,
           height=ySize +0.3 - (2 * wall_size), 
           width=zSize - (2 * wall_size) + 0.1,
           interpolate=False, frame=f, material=b_wall)
# back_frame = box(pos=(xMin, yMiddle, zMiddle), length=wall_size,
#                  height=ySize + 0.2, width=zSize, interpolate=False, frame=f,
#                  material=b_wall)
right = box(pos=(xMiddle, yMax + 0.1, zMiddle), length=wall_size, height=xSize,
            width=zSize, interpolate=False, frame=f, material=bg)

left.rotate(angle=pi / 2, axis=(0, 0, 1), 
            origin=(xMiddle, yMin - 0.1, zMiddle))
right.rotate(angle=-pi / 2,axis=(0, 0, 1),
             origin=(xMiddle, yMax + 0.1, zMiddle))

# Calibration box
calib = box(pos=(xMax - 0.2, yMax - 0.4, zMin + wall_size), length=0.1,
            height=0.1, width=0.1, color=color.blue, opacity=0.5, frame=f)
# Guiding Sphere
guide = sphere(pos=(1, 1, -0.1), radius=1.5, color=color.yellow, opacity=0.2,
               frame=f)

safe_sp = sphere(pos=(1, 1, -0.35), radius=0.13, color=color.green,
                 opacity=0.6, frame=f)

safe_sp.visible = False
# Score, timer and Instructions abel
score = label(pos=(xMax, yMax, zMax - 0.4), text="0", height=30, font="sans", 
              box=False)
timer = label(pos=(xMax - 1.8, yMax, zMax -0.4), text="0", height=30,
              font="sans", box=False)
instr = label(pos=(xMax, yMin + 0.8, zMax - 0.4), 
              text='Hit "Shift + Backspace" \nonce you are ready to calibrate',
              height=50, font="serif", box=False, frame=f)
speed_txt = label(pos=(xMax, yMax - 0.2, zMax -0.5), text="Speed", height=20,
                  font="sans", box=False, frame=f)
delay_txt = label(pos=(xMax, yMax - 0.2, zMax -0.55), text="Delay", height=20,
                  font="sans", box=False, frame=f)
force_txt = label(pos=(xMax, yMax - 0.2, zMax - 0.6), text="Force", height=20,
                  font="sans", box=False, frame=f)
float_txt = label(pos=(xMax, yMax - 0.2, zMax -0.65), text="Float", height=20,
                  font="sans", box=False, frame=f)

# Tool Position
ep = sphere(pos=(0, 0, 0), radius=0.05, frame=f, material=h_icon)
ep_sh = box(axis=(0, 0, 1), pos=(xMiddle, xMiddle, zMin + wall_size),
            length=0.0000000001, height=0.14, width=0.15, frame=f,
            material=sh_icon)
sp_sh = box(axis=(0, 0, 1), pos=(0, 0, 0), length=0.0000000001, height=0.24,
            width=0.25, frame=f, material=sh_icon)
safe_sh = box(axis=(0,0,1), pos=(0, 0, 0), length=0.0000000001, height=0.30,
              width=0.31, frame=f, material=sh_icon)
speed = med_speed
delay = medium_delay
force = low_force
gain_act = no_act
quit_flg = False

calMin_x = xMin
calMin_y = yMin
calMax_x = xMax
calMax_y = yMin
calMin_z = zMin + wall_size
calMax_z = zMax

# Method to perform the calibration of the workspace
def safe_calibration(pos, rad):
    instr.visible = True
    instr.text = "Move inside the green sphere once you are ready."
    safe_sp.visible = False
    safe_sp.pos = pos
    safe_sp.radius = rad
    safe_sp.visible = True
    safe_sh.visible = True
    safe_sh.pos = (pos[0], pos[1], zMin + wall_size)
    while True:
        msg = struct.unpack(MSG_FORMAT_RECV, sock.recv(MSG_SIZE_RECV))
        ep.pos = (msg[0], msg[1], msg[2])
        ep_sh.pos = (ep.pos[0], ep.pos[1], zMin + wall_size)
        if scene.kb.keys:
            s = scene.kb.getkey()
            if s == 'up':
                # Increase gravity assist
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale,
                          increment, speed, no_force, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act,
                          speed, no_force, delay, force))
            elif s == 'down':
                # Decrease gravity assist
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale,
                          decrement, speed, no_force, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act,
                          speed, no_force, delay, force))
            elif s == 'delete':
                # Turn off gravity assist
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, zeroes,
                          speed, no_force, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act,
                          speed, no_force, delay, force))
        if numpy.linalg.norm(numpy.subtract(safe_sp.pos, numpy.array([msg[0], msg[1], msg[2]]))) < abs(safe_sp.radius - ep.radius):
            safe_sp.visible = False
            safe_sh.visible = False
            return

def calibrate():
    while True:
        msg = struct.unpack(MSG_FORMAT_RECV, sock.recv(MSG_SIZE_RECV))
        global calMin_x, calMin_y, calMax_x, calMax_y, calMin_z, calMax_z
        calMin_x = calib.pos[0] - (calib.length / 2)
        calMin_y = calib.pos[1] - (calib.height / 2)
        calMax_x = calib.pos[0] + (calib.length / 2)
        calMax_y = calib.pos[1] + (calib.height / 2)
        calMin_z = zMin + wall_size
        calMax_z = calib.pos[2] + (calib.width / 2)
        if scene.kb.keys:
            s = scene.kb.getkey()
            if s == 'backspace':
                if abs(calMax_x - calMin_x) <= 0.15 or abs(calMax_y - calMin_y) <= 0.15 or abs(calMax_z - calMin_z) <= 0.15:
                    instr.text = "Workspace is too small to play"
                else:
                    sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale,
                              no_act, speed, w_calib, delay, force))
                    break
            if s == 'up':
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale,
                          increment, speed, w_calib, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12, no_scale, no_act,
                          speed, w_calib, delay, force))
            if s == 'down':
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale,
                          decrement, speed, w_calib, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act,
                          speed, w_calib, delay, force))
            elif s == 'delete':
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, zeroes,
                          speed, w_calib, delay, force))
                sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act,
                          speed, w_calib, delay, force))
        ep.pos = (msg[0], msg[1], msg[2])
        ep_sh.pos = (ep.pos[0], ep.pos[1], zMin + wall_size)
        if (msg[0] < calMin_x):
            calMin_x = msg[0]
        if (msg[0] > calMax_x and msg[0] < 0.6):
            calMax_x = msg[0]
        if msg[13] == 1:
            if (msg[1] < calMin_y):
                calMin_y = msg[1]
            if (msg[1] > calMax_y):
                calMax_y = msg[1]
        else:
            if (msg[1] < calMin_y):
                calMin_y = msg[1]
            if (msg[1] > calMax_y and msg[1] < 0.2):
                calMax_y = msg[1]
        if (msg[2] > calMax_z):
            calMax_z = msg[2]
        length = numpy.linalg.norm(numpy.subtract((calMin_x, calMin_y),
                                                  (calMax_x, calMin_y)))
        height = numpy.linalg.norm(numpy.subtract((calMin_x, calMin_y),
                                                  (calMin_x, calMax_y)))
        width = numpy.linalg.norm(numpy.subtract((calMin_x, calMin_y, calMin_z), (calMin_x, calMin_y, calMax_z)))
        calib.length = length
        calib.height = height
        calib.width = width
        calib.pos = (calMin_x + length / 2, calMin_y + height / 2, calMin_z + width / 2)

def init_setup():
    global calMin_x, calMin_y, calMax_x, calMax_y, calMin_z, calMax_z, force
    instr.visible = True
    instr.text = "Move inside the green sphere once you are ready."
    sock.send(struct.pack(MSG_FORMAT_SND, 0.12,no_scale,no_act,speed,no_force,delay,force))
    safe_pos = numpy.array([calib.pos[0], calib.pos[1],zMin + wall_size + safe_sp.radius + 0.02])
    safe_calibration(safe_pos, 0.13)
    instr.text = "Pop the sphere "
    force = zero_force
    counter = timeout
    sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,no_act,speed,game,delay,force))  # Notify that the calibration is to be started
    guide.visible = False

def cbt():
    instr.visible = True
    instr.text = "Press \"Spacebar\" to calibrate"
    while True:
        msg = struct.unpack(MSG_FORMAT_RECV, sock.recv(MSG_SIZE_RECV))
        ep.pos = (msg[0], msg[1], msg[2])
        ep_sh.pos = (ep.pos[0], ep.pos[1], zMin + wall_size)
        if scene.kb.keys:
            s = scene.kb.getkey()
            if s == ' ' and msg[0] < 0.6 and msg[1] > -0.5:
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_not_ended,no_act,speed,w_calib,delay,force))  # Notify that the calibration is to be started
                calib.pos = ep.pos
                calib.length = 0.1
                calib.height = 0.1
                calib.width = 0.1
                calib.visible = True
                instr.visible = True
                instr.text = "Define workspace and press \"Backspace\""
                calibrate()
                calib.visible = False
                return
            elif s == 'up':
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,increment,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,no_act,speed,w_calib,delay,force))
            elif s == 'down':
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,decrement,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,no_act,speed,w_calib,delay,force))
            elif s == 'delete':
                #print 'delete'
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,zeroes,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,cb_ended,no_act,speed,w_calib,delay,force))
for_txt = "Force: Off"
del_txt = "Delay: Medium"
spd_txt = "Speed: Medium"
def start():
    guide.visible = False  # Debug
    sp.visible = False
    sp_sh.visible = False
    safe_sh.visible = False
    calib.visible = False
    score.visible = False
    timer.visible = False
    speed_txt.visible = False
    delay_txt.visible = False
    force_txt.visible = False
    instr.visible = False
    float_txt.visible = False
    sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, no_act, speed,
                          mag_floor, delay, force))
    prev_pos = sp.pos
    new_pos = sp.pos
    # Number of iterations
    cbt()
    init_setup()
    score.visible = True
    timer.visible = True
    speed_txt.visible = True
    delay_txt.visible = True
    force_txt.visible = True
    float_txt.visible = True
    if speed == hi_speed:
        spd_txt = "Speed:   High"
    elif speed == med_speed:
        spd_txt = "Speed: Medium"
    elif speed == low_speed:
        spd_txt = "Speed:    Low"
    if force == low_force:
        for_txt = "Assist:     Low On"
    elif force == medium_force:
        for_txt = "Assist:     High On"
    elif force == zero_force:
        for_txt = "Assist:    Off"
    if delay == high_delay:
        del_txt = "Delay:   Long"
    elif delay == medium_delay:
        del_txt = "Delay: Medium"
    elif delay == low_delay:
        del_txt = "Delay:    Short"
# to generate a new position that is atleast a diameter far from the older
# one, and to mitigate the issue of a continuous issual of true for
# "attached".
guide.visible = False  # Debug
sp.visible = False
sp_sh.visible = False
safe_sh.visible = False
calib.visible = False
score.visible = False
timer.visible = False
speed_txt.visible = False
delay_txt.visible = False
force_txt.visible = False
instr.visible = False
float_txt.visible = False
sock.send(struct.pack(MSG_FORMAT_SND, 0.12, no_scale, zeroes, speed, no_force,
                      delay, force))
safe_pos = numpy.array([xMax - 0.2, yMiddle + 0.4,
                        zMin + wall_size + safe_sp.radius + 0.3])
safe_calibration(safe_pos, 0.13)
start()

while True:
    msg = struct.unpack(MSG_FORMAT_RECV, sock.recv(MSG_SIZE_RECV))
    ep.pos = (msg[0], msg[1], msg[2])
    ep_sh.pos = (ep.pos[0], ep.pos[1], zMin + wall_size)
    guide.pos = sp.pos
    attached = (bool)(msg[3])
    it_wait = (bool)(msg[11])
    wait = (bool)(msg[14])
    guide.radius = msg[4]
    score.text = "Score\n   " + str(int(msg[5]))
    game_pause = (bool)(msg[6])
    sp_pos = (msg[7], msg[8], msg[9])
    instr.visible = False
    speed_txt.text = spd_txt
    delay_txt.text = del_txt
    force_txt.text = for_txt
    flt = msg[12]
    if flt < 0.2:
        flt = 0
    float_txt.text = "Float:   " + str(flt * 100) + "%"
    if wait:
        sp.visible = False
    else:
        sp.visible = True
    if quit_flg == True:
        instr.visible = True
        break
    if msg[10] < 0:
        timer.text = "Timer\n" + "  0"
    else:
        lead = str(int(msg[10]/60))
        trail = int(msg[10]) - (int(msg[10]/60))* 60
        if trail < 10:
           trail = "0"+str(trail)
        else:
            trail = str(trail)
        timer.text = "Timer\n" + lead + ":" + trail
    if scene.kb.keys:  # If any input was pressed
        s = scene.kb.getkey()
        if s == 'q':  # Quit the program
                instr.visible = True
                instr.text = "Thank you for playing"
                sp.visible = False
                sp_sh.visible = False
                calib.visible = False
                score.visible = False
                timer.visible = False
                speed_txt.visible = False
                delay_txt.visible = False
                force_txt.visible = False
                float_txt.visible = False
                ep.visible = False
                ep_sh.visible = False
                instr.visible = True
                sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,no_act,speed,quit,delay,force))
                quit_flg = True
                continue
        elif s == 'r':
                start()
                sp.opacity = 0
                sp_sh.opacity = 0
                continue
        if game_pause:  # If the game was paused by the server
            if s == 'n':  # Instruct the server to restart a new session
                sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,no_act,speed,game_new_session,delay,force))
                game_mode = False
                sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,no_act,speed,game,delay,force))
            elif s == 'up':
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,increment,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,no_act,speed,w_calib,delay,force))                
            elif s == 'down':
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,decrement,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,no_act,speed,w_calib,delay,force))
            elif s == 'delete':
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,zeroes,speed,w_calib,delay,force))
                sock.send(struct.pack(MSG_FORMAT_SND,0.12,scale,no_act,speed,w_calib,delay,force))
        # If the game was not paused then check only for guidance sphere's
        # speed alterations and update accordingly
        else:
            if s == '1':
                speed = low_speed
                spd_txt = "Speed:    Low"
            elif s == '2':
                speed = med_speed
                spd_txt = "Speed: Medium"
            elif s == '3':
                speed = hi_speed
                spd_txt = "Speed:   High"
            elif s == '4':
                delay = high_delay
                del_txt = "Delay:   Long"
            elif s == '5':
                delay = medium_delay
                del_txt = "Delay: Medium"
            elif s == '6':
                delay = low_delay
                del_txt = "Delay:  Short"
            elif s == '7':
                force = low_force
                for_txt = "Assist: Low On"
            elif s == '8':
                force = medium_force
                for_txt = "Assist: High On"
            elif s == '0':
                force = zero_force
                for_txt = "Assist:    Off"
            elif s == 'up':
                gain_act = increment
            elif s == 'down':
                gain_act = decrement
            elif s == 'delete':
                gain_act = zeroes
            sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,gain_act,speed,game,delay,force))
            gain_act = no_act
            sock.send(struct.pack(MSG_FORMAT_SND,sp.radius,scale,gain_act,speed,game,delay,force))

    if game_pause == true:  # The game was paused
        instr.visible = True
        instr.text = "\'n\'- new game,\n \'r\'- restart, \'q\'- quit"
        sp.visible = False
        sp_sh.visible = True
    else:
        if it_wait == True:
            sp.visible = False
            sp_sh.visible = False
            sp.opacity = sp.opacity - 0.001
            sp_sh.opacity = sp_sh.opacity - 0.001
            sp.visible = True
            sp_sh.visible = True
        else:
            sp.visible = False
            sp_sh.visible = False
            sp.opacity = 0.6
            sp_sh.opacity = 0.6
            sp.pos = sp_pos
            sp_sh.pos = (sp.pos[0], sp.pos[1], zMin + wall_size)
            sp.visible = True
            sp_sh.visible = True
sock.close()
