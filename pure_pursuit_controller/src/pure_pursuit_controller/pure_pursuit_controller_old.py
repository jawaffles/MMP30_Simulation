#! /usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Twist
import actionlib
import tf
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped, PointStamped, QuaternionStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveBaseActionServer(object):
	
	def __init__(self, name):
		# create messages that are used to publish feedback/result
		self.__feedback = MoveBaseActionFeedback()
		self.__result = MoveBaseActionResult()
		self.__odom=None
		self.__action_name = name
		self.__as = actionlib.SimpleActionServer(self.__action_name, MoveBaseAction, execute_cb=self.__execute, auto_start = False)
		
		self.__odometrySub=rospy.Subscriber("/odometry", Odometry, self.__odometryCallback, queue_size=1)
		self.__simpleGoalSub=rospy.Subscriber("move_base_simple/goal", PoseStamped, self.__simpleGaolCallback, queue_size=1)
		self.__cmdVelPub=rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.__targetPointPub=rospy.Publisher("move_base/target_point", PointStamped, queue_size=1)
		self.__pathPub=rospy.Publisher("move_base/path", Path, queue_size=1)
		
		
		self.__lookahead=rospy.get_param('~lookahead',1)
		self.__speed_propotional_gain=rospy.get_param('~speed_propotional_gain', 5)
		
		self.__acc_lim_x=float(rospy.get_param('~acc_lim_x', 2.5))
		self.__acc_lim_y=float(rospy.get_param('~acc_lim_y', 2.5))
		self.__acc_lim_theta=float(rospy.get_param('~acc_lim_theta', 3.2))
		self.__max_vel_x=float(rospy.get_param('~max_vel_x', 1))
		self.__max_vel_theta=float(rospy.get_param('~max_vel_theta', 1))
		self.__min_vel_theta=float(rospy.get_param('~min_vel_theta', -1))
		self.__min_in_place_vel_theta=float(rospy.get_param('~min_in_place_vel_theta', 0.04))
		self.__yaw_goal_tolerance=float(rospy.get_param('~yaw_goal_tolerance', 0.05))
		self.__xy_goal_tolerance=float(rospy.get_param('~xy_goal_tolerance', 0.1))
		self.__robot_base_frame=rospy.get_param('~robot_base_frame', "base_link")
		self.__controller_frequency=float(rospy.get_param('~controller_frequency', 20))
		self.__dt=1.0/self.__controller_frequency
		self.__transform_tolerance=rospy.Duration(float(rospy.get_param('~transform_tolerance', 1)))
		self.__tfListener=tf.TransformListener(self.__transform_tolerance)	
		self.__as.start()	
	# The line following will have three phase, first rotate the robot in place to point to the goal 
	# second is the pure pursuit controlller to follow the path until it close to the goal position
	# third is to rotate in place to align with the desired pose 
	def __execute(self, goal):
		self.__control(goal,False)
	def __simpleGaolCallback(self, goal):
		self.__control(goal,True)

	def __control(self, goal, ifSimpleGoal):
		rospy.loginfo("Received a goal")
		# set controller frequency
		rate = rospy.Rate(self.__controller_frequency)
		rospy.logdebug(self.__controller_frequency)
		success = True
		if ifSimpleGoal:
			goal_frame=goal.header.frame_id
		else:
			goal_frame=goal.goal.target_pose.header.frame_id

		START=1
		MIDDLE=2
		END=3
		cmd_vel=Twist()
		
		# Wait for transformation from 
		# check if the tf from robot_base_frame to goal_frame exists
		# if not abort action
		now=rospy.Time.now()

		try:
			self.__tfListener.waitForTransform(goal_frame, self.__robot_base_frame, now, self.__transform_tolerance)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.InvalidTransform, tf.Exception) as exp:
			rospy.logwarn("%s" % exp)
			success = False
			if not ifSimpleGoal:
				self.__as.setAborted()
				self.__feedback.status.status=GoalStatus.ABORTED
				self.__as.publish_feedback(self.__feedback)
			else:
				rospy.logwarn("Goal failed due to transformation error.")
			return	

		#rospy.loginfo('transformation is ok')	
		path=Path()
		robotPose=PoseStamped()
		robotPose.header.stamp=now
		robotPose.header.frame_id=self.__robot_base_frame
		robotPose.pose.orientation.w=1		
		# startPosition, endPosition and path_slop will define the path in the goal frame
		startPose=self.__tfListener.transformPose(goal_frame, robotPose)
		startPosition=startPose.pose.position
		
		path.header.stamp=now
		path.header.frame_id=goal_frame
		path.poses.append(startPose)
		

		
		if ifSimpleGoal:
			endPosition=goal.pose.position
			endPose=goal
			q = (goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
		else:
			endPose=goal.goal.target_pose
			endPosition=goal.goal.target_pose.pose.position
			q = (goal.goal.target_pose.orientation.x, goal.goal.target_pose.orientation.y, goal.goal.target_pose.orientation.z, goal.target_pose.pose.orientation.w)

		path.poses.append(endPose)
		self.__pathPub.publish(path)
		path_slop=math.atan2(endPosition.y-startPosition.y, endPosition.x-startPosition.x)		
		# calculate the goal heading
		(roll, pitch, target_heading) = euler_from_quaternion(q)
		if not ifSimpleGoal:			
			self.__feedback.header.frame_id=goal_frame
			self.__feedback.status.goal_id=goal.goal_id
		state=START		
		forwardSpeed=0
		# start executing the action
		target_on_path=(1 if path_slop>-math.pi/2 and path_slop<math.pi/2 else -1) * self.__lookahead
		while (True):
			now=rospy.Time.now()
			if not ifSimpleGoal:
				self.__feedback.header.stamp=now
				# check that preempt has not been requested by the client
				if self.__as.is_preempt_requested():
					rospy.loginfo('%s: Preempted' % self.__action_name)
					self.__as.set_preempted()
					self.__feedback.status.status=GoalStatus.PREEMPTED
					success = False
					break	
			try:
				self.__tfListener.waitForTransform(self.__robot_base_frame, goal_frame, now, self.__transform_tolerance)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.InvalidTransform, tf.Exception) as exp:
				rospy.logwarn("%s" % exp)
				if not ifSimpleGoal:
					self.__feedback.status.status=GoalStatus.ABORTED
					self.__as.setAborted()

				rospy.logwarn("Goal failed due to transformation error.")
				success = False
				break
			# transform robot pose to goal frame
			robotPose.header.stamp=now
			currentRobotPose=self.__tfListener.transformPose(goal_frame, robotPose)
			# get robot orientation in goal frame
			currentRobotOrientation=currentRobotPose.pose.orientation
			# get robot positon in goal frame
			currentRobotPosition=currentRobotPose.pose.position
			in_place_rotation_vel=self.__min_in_place_vel_theta
			
			if state==START or state==END:
				# get yaw of the robot
				q = (currentRobotOrientation.x, currentRobotOrientation.y, currentRobotOrientation.z, currentRobotOrientation.w)
				(roll, pitch, robot_heading) = euler_from_quaternion(q)
				# calculate the error in heading
				if state==START:
					heading_error=self.angleDiff(path_slop,robot_heading)
				else:
					heading_error=self.angleDiff(target_heading,robot_heading)			
				rospy.loginfo ("heading error is %f" % heading_error)	
				# if the error is larger than the tolerance
				if abs(heading_error)>self.__yaw_goal_tolerance:
						
					cmd_vel.linear.x=0
					cmd_vel.linear.y=0
					cmd_vel.linear.z=0
					cmd_vel.angular.x=0
					cmd_vel.angular.y=0					
					new_in_place_rotation_vel = heading_error*self.__speed_propotional_gain
					rotateAcc=(new_in_place_rotation_vel-in_place_rotation_vel)/self.__dt
					# check if the anglar speed acceleration larger than the limit
					if abs(rotateAcc) > self.__acc_lim_theta:
						in_place_rotation_vel=in_place_rotation_vel+(-1 if new_in_place_rotation_vel<0 else 1)*self.__acc_lim_theta*self.__dt
					else:
						if abs(new_in_place_rotation_vel) < self.__min_in_place_vel_theta:
							in_place_rotation_vel = (-1 if new_in_place_rotation_vel<0 else 1) * self.__min_in_place_vel_theta
						else:
							in_place_rotation_vel = new_in_place_rotation_vel					
					cmd_vel.angular.z=in_place_rotation_vel	
					rospy.logdebug ("Turn in place, speed is %f" % in_place_rotation_vel)			
				# error is smaller than the tolerance, start the pure pursuit controller		
				else:
					cmd_vel.angular.z=0
					if state==START:
						state=MIDDLE
						rospy.logdebug ("start pure pursuit")				
					else:
						success = True
						rospy.loginfo('Gaol reached')
						if not ifSimpleGoal:
							self.__feedback.status.status=GoalStatus.SUCCEEDED
							self.__as.set_succeeded(self.__result)
							# publish the feedback
							self.__as.publish_feedback(self.__feedback)
						break
			# perform the pure pursuit
			else:
				distToGoal2=(currentRobotPosition.x-endPosition.x)*(currentRobotPosition.x-endPosition.x)+(currentRobotPosition.y-endPosition.y)*(currentRobotPosition.y-endPosition.y)
				rospy.logdebug("distance to goal is %f" % math.sqrt(distToGoal2))
				# reach goal position
				if distToGoal2 < self.__xy_goal_tolerance*self.__xy_goal_tolerance:
					rospy.logdebug("reached goal position, state is END")
					state=END
					cmd_vel.linear.x=0
				else:
					# first calculate the target point
					target=PointStamped()
					target.header.frame_id=goal_frame
					target.header.stamp=now
					k=math.tan(path_slop)
					a=1+k*k
					x0=(startPosition.x - currentRobotPosition.x)
					y0=(startPosition.y - currentRobotPosition.y)
					b=2*x0 + 2*k*y0
					c=x0*x0 + y0*y0-self.__lookahead*self.__lookahead
					d=b*b-4*a*c
					if d>=0:
						t1=(-b+math.sqrt(d))/(2*a)
						t2=(-b-math.sqrt(d))/(2*a)
						if abs(t1)>=abs(target_on_path):
							target_on_path=t1
						else:
							target_on_path=t2
						rospy.logdebug("target on path is %f, t1 is %f, t2 is %f" % (target_on_path, t1, t2))
					else:
						target_on_path=target_on_path+(1 if path_slop>-math.pi/2 and path_slop<math.pi/2 else -1)*cmd_vel.linear.x*self.__dt
						rospy.logwarn("Can't find target on the path")
					if abs(target_on_path)>abs(startPosition.x-endPosition.x):
    							target_on_path=endPosition.x-startPosition.x
					target.point.x=startPosition.x+target_on_path
					target.point.y=startPosition.y+target_on_path*k

					self.__targetPointPub.publish(target)

					targetInRobotFrame=self.__tfListener.transformPoint(self.__robot_base_frame, target).point
					cmd_vel.linear.y=0
					cmd_vel.linear.z=0
					cmd_vel.angular.x=0
					cmd_vel.angular.y=0	
					desiredAngularSpeed=2*targetInRobotFrame.y/(self.__lookahead*self.__lookahead)*self.__max_vel_x
					rospy.logdebug("Desired anglar speed is %f" % desiredAngularSpeed)
					#first check if the acceleration is larger than the max limit
					acc=(desiredAngularSpeed-cmd_vel.angular.z)/self.__dt
					if abs(acc)>self.__acc_lim_x:
						cmd_vel.angular.z=cmd_vel.angular.z+(-1 if acc<0 else 1)*self.__acc_lim_theta*self.__dt
					else:
						cmd_vel.angular.z=desiredAngularSpeed
					rospy.logdebug("Angular speed is %f" % cmd_vel.angular.z)
					# calculate the distance to stop the robot under the new forwardSpeed and maximum deceleration
					stopDist=self.__stopDistance(cmd_vel.linear.x, self.__acc_lim_x)
					rospy.logdebug("stop distance is %f" % stopDist)
					# if stop distance is larger than the distance to the goal, need to start declerating
					if stopDist*stopDist > distToGoal2:
						rospy.logdebug("start to decelerate")
						cmd_vel.linear.x=cmd_vel.linear.x-self.__acc_lim_x*self.__dt
						cmd_vel.linear.x=(0 if cmd_vel.linear.x<0 else cmd_vel.linear.x)
						rospy.logdebug("Decrease forward speed to %f" % cmd_vel.linear.x)
					else:
						# increase the speed using the largest acceleration
						cmd_vel.linear.x=cmd_vel.linear.x+self.__acc_lim_x*self.__dt
						rospy.logdebug("increase forward speed to %f" % cmd_vel.linear.x)
					if not ifSimpleGoal:			
						self.__feedback.status.status=GoalStatus.ACTIVE
						self.__feedback.feedback.base_position=currentRobotPose
					rospy.logdebug("forward speed is %f" % cmd_vel.linear.x)
			# put the speeds into their limits
			cmd_vel.linear.x=(self.__max_vel_x if cmd_vel.linear.x>self.__max_vel_x else cmd_vel.linear.x)
			cmd_vel.angular.z=(self.__max_vel_theta if cmd_vel.angular.z>self.__max_vel_theta else cmd_vel.angular.z)
			cmd_vel.angular.z=(self.__min_vel_theta if cmd_vel.angular.z<self.__min_vel_theta else cmd_vel.angular.z)
			# publish cmd_vel message
			self.__cmdVelPub.publish(cmd_vel)
			# publish the feedback
			if not ifSimpleGoal:
				self.__as.publish_feedback(self.__feedback)
			rate.sleep()	
	def __odometryCallback(self, data):
		self.__odom=data
	def angleDiff(self, a1, a2):
		da = a1 - a2
		da -= (2*math.pi if da > math.pi else 0)
		da += (2*math.pi if da < -math.pi else 0)
		return da		
	# calculate how the distance to stop the robot
	def __stopDistance(self, v, acc):
		return v*v/acc*2

def pure_pursuit_controller():
	rospy.init_node('move_base')
	server = MoveBaseActionServer(rospy.get_name())
	rospy.spin()
		
if __name__ == '__main__':
	pure_pursuit_controller()

