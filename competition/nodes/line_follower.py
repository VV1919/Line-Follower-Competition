#!/usr/bin/env python
from __future__ import division

import rospy
import math
import time
import numpy
from rosrider_lib.rosrider import ROSRider

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion


class LineFollower:

    def __init__(self):

        # initialize robot, giving name, update rate, and process_image  callback handler
        self.robot = ROSRider('robot1', 20.0)
        # this array will hod the brigtness data from the sensor
        self.sensor_row = []

        # this flag is true, only when robot is cleared to start
        self.started = False

        # NOTICE: odometry data can be accessed by
        # self.robot.x, self.robot.y, self.robot.yaw, self.robot.vel_x, and self.robot.vel_z

        # ENTER YOUR INITIALIZATION HERE
        self.prev_rot = 1

        rot_min_max = 3
        array_size=32
        self.rot_values = numpy.linspace(rot_min_max,-rot_min_max,array_size)
        vel_max = 1.2
        vel_min = 0.85
        self.vel_values = numpy.append(numpy.linspace(vel_min,vel_max,16),numpy.linspace(vel_max,vel_min,16))
        # wait for evaluator, do not remove
        rospy.wait_for_message('/simulation_metrics', String)

        # grace period, do not remove
        time.sleep(0.5)
        
        self.started = True

        self.a = 0

    def main(self):
        # program will process incoming callbacks, namely process_image, and process_odom until shutdown.
        while self.robot.is_ok():
            # if robot is shutting down, or not yet started, do not process image
            if self.robot.is_shutdown or not self.started:
                return
            # reinitialize the array each time, new image arrives
            self.sensor_row = []

            image = self.robot.get_image()

            # NOTICE: The below code is left as an example. It is the competitors duty, to make it function right.
            # Or you may develop a better method that locks to yellow
            # The code below calculates brightness per pixel, and append it in array sensor_row
            for i in range(image.width):
                brightness = (0.2126 * ord(image.data[i * 3])) + (0.7152 * ord(image.data[i * 3 + 1])) + (0.0722 * ord(image.data[i * 3 + 2]))
                self.sensor_row.append(brightness)

            # ENTER YOUR LINE DETECTING CODE HERE
            max = self.sensor_row[0]
            index = 0
            for i in range(0, len(self.sensor_row)):
                if self.sensor_row[i] > max:
                    max = self.sensor_row[i]
                    index = i

            # error = index - (image.width / 2)
            
            # ENTER YOUR STEERING CODE HERE
            if self.sensor_row[index] == 0:

                self.robot.rotate(self.prev_rot)
                self.robot.move(self.vel_values[index])

            else:
                self.robot.rotate(self.rot_values[index])
                self.robot.move(self.vel_values[index])
                self.prev_rot = self.rot_values[index]

        # rospy.spin has finished waiting, program is shutdown, so send stop to robot.
        self.robot.stop()


if __name__ == '__main__':
    try:
        node = LineFollower()
        node.main()
    except rospy.ROSInterruptException:
        pass
