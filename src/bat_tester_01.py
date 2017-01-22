#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import BatteryState

def batRead():
    bat_dir = "/sys/devices/platform/7000c400.i2c/i2c-1/1-0042/iio_device/"
    volt0_in = open(bat_dir + "in_voltage0_input")
    curr0_in = open(bat_dir + "in_current0_input")

    rospy.init_node('BatRead')
    pub = rospy.Publisher('jetson_battery', BatteryState, queue_size=20)

    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        try:
            # Read voltage in mV, store in V
            voltage = float(volt0_in.read().strip()) / 1000
            volt0_in.seek(0)

            # Read voltage in mA, store in A
            current = float(curr0_in.read().strip()) / 1000
            curr0_in.seek(0)
        except (IOError, ValueError) as e:
            rospy.logerr("I/O error: {0}".format(e))
        else:
            bat_msg = BatteryState()
            bat_msg.header.stamp = rospy.Time.now()
            bat_msg.voltage = voltage
            bat_msg.current = -current
            bat_msg.charge = float('NaN')
            bat_msg.capacity = float('NaN')
            bat_msg.design_capacity = float('NaN')
            bat_msg.percentage = float('NaN')
            bat_msg.power_supply_status = bat_msg.POWER_SUPPLY_STATUS_UNKNOWN
            bat_msg.power_supply_health = bat_msg.POWER_SUPPLY_HEALTH_UNKNOWN
            bat_msg.power_supply_technology = bat_msg.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            bat_msg.present = True

            rospy.logdebug('New battery message: %s'%bat_msg)
            pub.publish(bat_msg)

        rate.sleep()

if __name__ == '__main__':
    batRead()
