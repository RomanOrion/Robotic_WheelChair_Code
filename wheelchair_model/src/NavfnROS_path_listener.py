#!/usr/bin/env python

import rospy 
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def callback(msg):
    print("called")

    localGoal = msg.poses[1]
    pathIndexLength = len(msg.poses)
    print(pathIndexLength)

    currentX = msg.poses[0].pose.position.x
    currentY = msg.poses[0].pose.position.y

    fianlX = msg.poses[pathIndexLength-1].pose.position.x
    finalY = msg.poses[pathIndexLength-1].pose.position.y

    distanceToFinalGoal = math.sqrt((fianlX-currentX)**2 + (finalY-currentY)**2)

    if (distanceToFinalGoal <= 1.0):

        localGoalPublisher.publish(msg.poses[pathIndexLength-1])

    else:

        for position in msg.poses:

            localX = position.pose.position.x
            localY = position.pose.position.y

            distanceToGoal = math.sqrt((localX-currentX)**2 + (localY-currentY)**2)

            #print(distanceToGoal)

            if (distanceToGoal >= 1.0):

                localGoalPublisher.publish(position)
                break
    

    print(localGoal)


    


#while(rospy.is_shutdown() == False):
if __name__ == '__main__':


    rospy.init_node("getLocalGoal", anonymous=False)
    rate = rospy.Rate(10)
    
    navigationPathSubscriber = rospy.Subscriber("/move_base/NavfnROS/plan", Path, callback)
    localGoalPublisher = rospy.Publisher('/local_goal', PoseStamped, queue_size=10)
    print("here")
    #rate.sleep()

    rospy.spin()

