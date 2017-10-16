#!/usr/bin/env python2

import rospy
import subprocess
from std_msgs.msg import String

def dataRead():
    
    ps_pub = rospy.Publisher('ps_stats', String, queue_size=20)
    top_pub = rospy.Publisher('top_stats', String, queue_size=20)
    tegra_pub = rospy.Publisher('tegra_stats', String, queue_size=20)
    rospy.init_node('dataRead')

    rate = rospy.Rate(10) #10 hz
    while not rospy.is_shutdown():
        

        output = subprocess.check_output(['top','-bn1'])
        top_pub.publish(output)

        #need to run chmod +x so permission is not denied
        subprocess.call(['chmod','+x','tegra.sh'])
        output = subprocess.check_output(['./tegra.sh'])
        tegra_pub.publish(output)

        #need to run chmod +x so permission is not denied
        subprocess.call(['chmod','+x','run.sh'])
        output = subprocess.check_output(['./run.sh'])
        ps_pub.publish(output)

        rate.sleep()

if __name__ == '__main__':
    dataRead()
