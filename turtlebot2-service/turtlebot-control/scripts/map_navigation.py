import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

class map_navigation():

  def __init__(self):
    # declare the coordinates of interest
    self.xPoint2 = 5.62812172448
    self.yPoint2 = 3.84335105296
    self.xPoint3 = 5.95316634027
    self.yPoint3 = 7.04917697049
    self.xPoint4 = 2.04774322225
    self.yPoint4 = 7.06378159836
    self.xPoint1 = 2.14230980471
    self.yPoint1 = 2.01097621391
    self.goalReached = False
    # initiliaze
    rospy.init_node('map_navigation', anonymous=False)
    self.Position=1
    self.chain='forward'
    while True:
      choice = self.next_position(self.Position)
      if (choice == 1):

        self.goalReached = self.moveToGoal(self.xPoint1, self.yPoint1)

      elif (choice == 2):

        self.goalReached = self.moveToGoal(self.xPoint2, self.yPoint2)

      elif (choice == 3):

        self.goalReached = self.moveToGoal(self.xPoint3, self.yPoint3)

      elif (choice == 4):

        self.goalReached = self.moveToGoal(self.xPoint4, self.yPoint4)

      if (choice!='q'):

        if (self.goalReached):
          rospy.loginfo("Congratulations!")
          #rospy.spin()

        else:
          rospy.loginfo("Hard Luck!")
         
  def next_position(self, position):
      if position==1:
          self.chain='forward'
          self.Position=self.Position+1
          return self.Position
      elif position==4:
          self.chain='backward'
          self.Position=self.Position-1
          return self.Position
      elif self.chain=='forward':
          self.Position=self.Position+1
          return self.Position
      elif self.chain=='backward':
          self.Position=self.Position-1
          return self.Position
      

  def shutdown(self):
      # stop turtlebot
      rospy.loginfo("Quit program")
      rospy.sleep()

  def moveToGoal(self,xGoal,yGoal):

      #define a client for to send goal requests to the move_base server through a SimpleActionClient
      ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

      #wait for the action server to come up
      while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
              rospy.loginfo("Waiting for the move_base action server to come up")


      goal = MoveBaseGoal()

      #set up the frame parameters
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()

      # moving towards the goal*/

      goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
      goal.target_pose.pose.orientation.x = 0.0
      goal.target_pose.pose.orientation.y = 0.0
      goal.target_pose.pose.orientation.z = 0.0
      goal.target_pose.pose.orientation.w = 1.0

      rospy.loginfo("Sending goal location ...")
      ac.send_goal(goal)

      ac.wait_for_result(rospy.Duration(60))

      if(ac.get_state() ==  GoalStatus.SUCCEEDED):
              rospy.loginfo("You have reached the destination")
              return True

      else:
              rospy.loginfo("The robot failed to reach the destination")
              return False

if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
