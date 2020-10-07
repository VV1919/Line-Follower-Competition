#!/usr/bin/env python
from __future__ import division

import rospy
import math
import time

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
        self.last_rotation = -1

        # wait for evaluator, do not remove
        rospy.wait_for_message('/simulation_metrics', String)

        # grace period, do not remove
        time.sleep(0.5)
        
        self.started = True

    def main(self):
        # program will process incoming callbacks, namely process_image, and process_odom until shutdown.
        while self.robot.is_ok():
            # if robot is shutting down, or not yet started, do not process image
            if self.robot.is_shutdown or not self.started:
                return
            # reinitialize the array each time, new image arrives
            self.sensor_row = []

            image = self.robot.get_image()

            # ENTER YOUR LINE DETECTING CODE HERE
            # find the center

        # ENTER YOUR STEERING CODE HERE

        # rospy.spin has finished waiting, program is shutdown, so send stop to robot.
        self.robot.stop()


if __name__ == '__main__':
    try:
        node = LineFollower()
        node.main()
    except rospy.ROSInterruptException:
        pass
