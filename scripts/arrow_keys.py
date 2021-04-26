#!/usr/bin/env python

import rospy
import pygame
from geometry_msgs.msg import TwistStamped

keys = [False, False, False, False] #[up, left, down, right]

pygame.init()
key_pad = pygame.display.set_mode((330, 330))
pygame.display.set_caption("Key Pad")

v, w = 0, 0

if __name__ == '__main__':

	rospy.init_node('arrow_keys')
	#pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size = 1)
	rate = rospy.Rate(30)

	speed = 0.5

	while not rospy.is_shutdown():

		for event in pygame.event.get():

			if event.type == pygame.QUIT:
            			pygame.quit() 

			if event.type == pygame.KEYDOWN: # The key is pressed.
				if event.key==pygame.K_UP:
				    keys[0]=True
				if event.key==pygame.K_LEFT:
				    keys[1]=True
				if event.key==pygame.K_DOWN:
				    keys[2]=True
				if event.key==pygame.K_RIGHT:
				    keys[3]=True

			if event.type == pygame.KEYUP:
				if event.key==pygame.K_UP:
				    keys[0]=False
				if event.key==pygame.K_LEFT:
				    keys[1]=False
				if event.key==pygame.K_DOWN:
				    keys[2]=False
				if event.key==pygame.K_RIGHT:
				    keys[3]=False

			if keys[0]:
				if keys[2]:
					v = 0
				else: v = speed
			elif keys[2]:
				if keys[0]:
					v = 0
				else: v = -speed
			else: v = 0

			if keys[1]:
				if keys[3]:
					w = 0
				else: w = speed
			elif keys[3]:
				if keys[1]:
					w = 0
				else: w = -speed
			else: w = 0

		msg = TwistStamped()
		msg.twist.linear.x = v
		msg.twist.angular.z = w
		pub.publish(msg)
		rate.sleep()
	pygame.quit()