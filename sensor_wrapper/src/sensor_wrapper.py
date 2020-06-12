#!/usr/bin/env python

import sys
import random
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates

class sensor_wrapper:
  def __init__(self):
    self.tractor_state_noisy_pub = rospy.Publisher("/ebs_tractor/state/noisy", String, queue_size=10)
    self.tractor_state_pub = rospy.Publisher("/ebs_tractor/state/ground_truth", String, queue_size=10)
    self.laser_angle_pub = rospy.Publisher("/ebs_tractor/laser/angles", String, queue_size=10)
    self.laser_dist_pub = rospy.Publisher("/ebs_tractor/laser/distance", String, queue_size=10)

    self.orientation_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_cb)
    self.laser_sub = rospy.Subscriber("/ebs_tractor/laser/scan", LaserScan, self.laser_cb)

  def state_cb(self,data):
    # find the index of the tractor
    index = data.name.index('ebs_tractor')
    x = data.pose[index].position.x
    y = data.pose[index].position.y
    theta = data.pose[index].orientation.z
    clean_state = "{} {} {}".format(x, y, theta)

    dev = 1.5
    x += random.uniform(-dev, dev)
    y += random.uniform(-dev, dev)
    theta += random.uniform(-dev, dev)
    dirty_state = "{} {} {}".format(x, y, theta)

    self.tractor_state_pub.publish(clean_state)
    self.tractor_state_noisy_pub.publish(dirty_state)

  def laser_cb(self, data):
    min_angle = data.angle_min
    max_angle = data.angle_max
    increment = data.angle_increment
    num_points = (max_angle - min_angle) / increment

    angles = np.linspace(min_angle, max_angle, num_points + 2)
    distance = np.asarray(data.ranges, dtype=np.float32)
    angles = np.array2string(angles, precision=5, separator=' ')
    distance = np.array2string(distance, precision=5, separator=' ')

    self.laser_angle_pub.publish(angles)
    self.laser_dist_pub.publish(distance)

def main(args):
  rospy.init_node('sensor_wrapper', anonymous=True)
  _ = sensor_wrapper()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)