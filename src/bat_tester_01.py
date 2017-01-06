#Things to do:
#open file and read its values from it, (voltage0 gives out 12344 =12.344 and current0=66=>.066
#file location:
#/sys/devices/platform/7000c400.i2c/i2c-1/1-0042/iio_device/(in_voltage0_input, in_current0_input)

import rospy
#from sensor_msgs.msg import BatteryState

try:
	file volt0_in=open("/sys/devices/platform/7000c400.i2c/i2c-1/1-0042/iio_device/in_voltage0_input",r)
	file curr0_in=open("/sys/devices/platform/7000c400.i2c/i2c-1/1-0042/iio_device/in_voltage0_input",r)


	print(volt0_in.read())
	print(curr0_in.read())

	from std_msgs.msg import String

	def batRead():
	    pub = rospy.Publisher('BatRead', String, queue_size=20)
	    rospy.init_node('BatRead', anonymous=True)
	    rate = rospy.Rate(5) # 5hz
	    while not rospy.is_shutdown():
		bat_str = "VOLTAGE" + volt0_in.read() + " " + "CURRENT"+ curr0_in.read()
		rospy.loginfo(bat_str)
		pub.publish(bat_str)
		rate.sleep()

	if __name__ == '__main__':
	    try:
		batRead()
	    except rospy.ROSInterruptException:
		pass

except IOError as e:
    print "I/O error({0}): {1}".format(e.errno, e.strerror)
