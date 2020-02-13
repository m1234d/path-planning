#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
#
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports B and C. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor C power will be controlled by the position of motor B. Manually rotate motor B, and motor C's power will change.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers
import math
import path

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

class Pid:
    def __init__(self, P, I, D):
        # initialize variables here
        self.kp = P
        self.ki = I
        self.kd = D
        self.last_error = 0
        self.acc = []
        self.last_acc = 0
        self.count = 0
        self.dist = 500

    def action(self, error, dt):
        # compute PID here
        #action = (abs(error)/error) * self.kp * error**2
        action = self.kp * error
        delta = (error - self.last_error) / dt
        action += self.kd * delta
        if abs(action) <= 10:
            self.last_error = error
            self.acc.append(error*dt)
            # if self.count > self.dist:
            #     self.acc.pop(0)
            self.count += 1


        action += self.ki * sum(self.acc)
        return action

def light():
    try:
        value = BP.get_sensor(BP.PORT_1)
        return value
    except brickpi3.SensorError as error:
        print(error)
        return -1

def on_path():
    value = light()
    if value > 2200:
        return 2
    elif value <= 2200 and value >= 2130:
        return 1
    else:
        return 0
def reset_encoders():
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))

def encoder1():
    return BP.get_motor_encoder(BP.PORT_A)

def encoder2():
    return BP.get_motor_encoder(BP.PORT_B)

def drive(motor1, motor2):
    if motor1 < 0:
        BP.set_motor_power(BP.PORT_A, 1.23*motor1 - 4)
        BP.set_motor_power(BP.PORT_B, 1.3*motor2 - 4)
    elif motor1 > 0:
        BP.set_motor_power(BP.PORT_A, 1.25*motor1 + 4) #1.18
        BP.set_motor_power(BP.PORT_B, 1.3*motor2 + 4)
    else:
        BP.set_motor_power(BP.PORT_A, 1.20*motor1)
        BP.set_motor_power(BP.PORT_B, 1.3*motor2)

def follow_light():
    BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.NXT_LIGHT_ON)
    reset_encoders()
    count = 0
    try:
        while True:
            path = on_path()
            if path == 2:
                count += 1
                drive(-20, 30)
            elif path == 1:
                drive(30, 30)
            else:
                if count > 10:
                    print(count)
                count = 0
                drive(30, -20)
            time.sleep(0.02)
    except KeyboardInterrupt:
        BP.reset_all()

def odometry(start, moves):
    reset_encoders()
    tstep = 0.02
    tcount = 0
    movecount = 0
    radius = 1.075
    width = 5.71 #higher = more turn
    prev_click_left = 0
    prev_click_right = 0
    prev_theta = 0
    prev_x = start[1]
    prev_y = start[0]
    sign = 1
    target_x = moves[movecount][0]
    target_y = moves[movecount][1]
    target_theta = moves[movecount][2]

    #thetaPid = Pid(500, 0.1, 1)
    thetaPid = Pid(230, .1, .3)
    distPid = Pid(20, 0, .5)
    #distPid = Pid(0, 0, 0)

    distThreshold = 0.05
    thetaThreshold = 0.005
    movingThreshold = 0.001
    goodcount = 0
    moveDone = False
    actuallyDone = False
    stopAngle = False
    anglecount = 0
    saved_target = 0
    endTime = 0
    offset = 0
    moveTimer = 0

    distError = math.sqrt(target_x**2 + target_y**2)
    if distError < 0:
        sign = -1

    try:
        while True:
            click_left = encoder1()*1.004 #higher = turns more left
            click_right = encoder2()
            angular_left = (click_left - prev_click_left) / tstep
            angular_right = (click_right - prev_click_right) / tstep
            angular_left = angular_left * math.pi/180
            angular_right = angular_right * math.pi/180

            v = radius * (angular_left + angular_right) / 2
            angular = radius * (angular_right - angular_left) / width

            k00 = v * math.cos(prev_theta)
            k01 = v * math.sin(prev_theta)
            k02 = angular

            k10 = v * math.cos(prev_theta + (tstep/2) * k02)
            k11 = v * math.sin(prev_theta + (tstep/2) * k02)
            k12 = angular

            k20 = v * math.cos(prev_theta + (tstep/2) * k12)
            k21 = v * math.sin(prev_theta + (tstep/2) * k12)
            k22 = angular

            k30 = v * math.cos(prev_theta + tstep * k22)
            k31 = v * math.sin(prev_theta + tstep * k22)
            k32 = angular

            x = prev_x + (tstep/6) * (k00 + 2 * (k10 + k20) + k30)
            y = prev_y + (tstep/6) * (k01 + 2 * (k11 + k21) + k31)
            theta = prev_theta + (tstep/6) * (k02 + 2 * (k12 + k22) + k32)
            prev_x = x
            prev_y = y
            prev_theta = theta
            prev_click_left = click_left
            prev_click_right = click_right

            ## PID

            #Get distance and theta from target_y
            align_theta = math.atan2(target_y - y, target_x - x)
            if abs(align_theta - theta) > math.pi:
                offset = 2*math.pi
            else:
                offset = 0
            align_theta = align_theta + offset
            thetaError = align_theta - theta

            distError = sign*(math.sqrt((target_x - x)**2 + (target_y - y)**2)*math.cos(thetaError))
            distAction = distPid.action(distError, tstep)

            if stopAngle:
                thetaError = saved_target - theta

            if moveDone:
                thetaError = target_theta - theta
                if abs(thetaError) > math.pi:
                    offset = 2*math.pi
                else:
                    offset = 0
                thetaError = thetaError + offset
                distAction = 0

            if abs(distError) > 5:
                thetaAction = thetaPid.action(thetaError*1, tstep)
            else:
                moveTimer += 1
                thetaAction = thetaPid.action(thetaError, tstep)

            #print(distError, thetaError*180/math.pi)
            print(align_theta*180/math.pi)
            print(theta*180/math.pi)
            print(thetaAction)
            print(" ")
            speedCap = 30
            if distAction > speedCap:
                distAction = speedCap
            elif distAction < -speedCap:
                distAction = -speedCap

            leftMotor = distAction - thetaAction
            rightMotor = distAction + thetaAction

            if leftMotor > speedCap:
                leftMotor = speedCap
            if leftMotor < -speedCap:
                leftMotor = -speedCap
            if rightMotor > speedCap:
                rightMotor = speedCap
            if rightMotor < -speedCap:
                rightMotor = -speedCap

            # if tcount == 0:
            #     leftMotor = leftMotor*.3
            #     rightMotor = rightMotor*.3

            # if tcount == 1:
            #     leftMotor = leftMotor*.6
            #     rightMotor = rightMotor*.6

            #print(leftMotor, rightMotor)
            drive(leftMotor, rightMotor)

            time.sleep(tstep)
            tcount+=1
            if moveDone:
                endTime += 1
                if abs(thetaError) <= thetaThreshold:
                    goodcount += 1
            else:
                if abs(distError) <= 5:
                    anglecount += 1
                if abs(distError) <= distThreshold and abs(thetaError) <= thetaThreshold:
                    goodcount += 1 

            if stopAngle == False and anglecount >= 5:
                print("STOPPED!")
                saved_target = align_theta
                stopAngle = True

            if goodcount >= 30 or moveTimer >= 100:
                if moveDone == False:
                    moveDone = True
                    drive(0, 0)
                    print("move done")
                    time.sleep(.5)
                if target_theta == None:
                    actuallyDone = True
                elif abs(target_theta - theta) <= thetaThreshold:
                    actuallyDone = True
                elif endTime >= 100:
                    actuallyDone = True


            if actuallyDone:
                print("DONE")
                tcount = 0
                moveTimer =0 
                endTime = 0
                movecount += 1
                anglecount = 0
                moveDone = False
                actuallyDone = False
                stopAngle = False
                goodcount = 0
                print(x)
                print(y)
                print((theta*180/math.pi))
                prev_x = target_x
                prev_y = target_y
                prev_theta = target_theta
                drive(0, 0)
                time.sleep(1)
                if movecount >= len(moves):
                    print("Final X: " + str(x))
                    print("Final Y: " + str(y))
                    print("Final Theta: " + str(theta*180/math.pi))
                    BP.reset_all()
                    return
                target_x = moves[movecount][0]
                target_y = moves[movecount][1]
                target_theta = moves[movecount][2]
                distError = math.sqrt((target_x - x)**2 + (target_y - y)**2)
                if distError < 0:
                    sign = -1

                align_theta = math.atan2(target_y - y, target_x - x)
                print(align_theta)
                print(theta)
                if abs(align_theta - theta) > 180:
                    offset = 2*math.pi
                else:
                    offset = 0


    except KeyboardInterrupt:
        BP.reset_all()

def main():
    # Step 1: Get path

    #(y, x) my y and my x
    start = (12, 6, 'South')
    #end = (13, 6, 'North')
    end = (12, 48, 'South')
    p = path.main(start, end)

    #p = [(30, 60, 1.5707963267948966), (30, 65, 0.0), (48, 65, -1.5707963267948966), (48, 60, 3.141592653589793)]

    print(p)
    
    #p = [(12, 12, 0), (24, 0, 0), (12, -12, 0), (0, 0, 0)]
    #p = [(24, 0, -math.pi/2)]
    # Step 2: Follow path
    odometry(start, p)
main()
