#!/usr/bin/env python2

import rospy
import subprocess
from std_msgs.msg import String

def system_stat_monitor():
    
    rospy.init_node('system_stat_monitor')

    #topic for string from ps_run
    ps_pub = rospy.Publisher('ps_stats', String, queue_size=20)
    #topic for string from top -bn1
    top_pub = rospy.Publisher('top_stats', String, queue_size=20)
    #topic for string from tegra.sh
    tegra_pub = rospy.Publisher('tegra_stats', String, queue_size=20)
    
    rate = rospy.Rate(10) #10 hz
    while not rospy.is_shutdown():
        
        output = subprocess.check_output(['top','-bn1'])
        top_pub.publish(output)

        output = subprocess.check_output(['./tegra.sh'])
        tegra_pub.publish(output)

        output = subprocess.check_output(['./ps_run.sh'])
        ps_pub.publish(output)

        rate.sleep()

if __name__ == '__main__':
    system_stat_monitor()
