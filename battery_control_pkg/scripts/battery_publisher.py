#! usr/bin/env python3

import sys
import rospy
import time
from std_msgs.msg import Int32, String, Float32, Bool, Header
from sensor_msgs.msg import Image
from battery_control_pkg.msg import BatteryStatus
from battery_control_pkg.Battery_Control import Battery_Control


class Battery():

    def __init__(self, batteryDevice):
        self.battery_publisher = rospy.Publisher(
            '/battery_status', BatteryStatus, queue_size=100)  # publisher for battery status
        self.msg = BatteryStatus()

        self.ctrl_c = False  # CREATE FLAG FOR CTRL+C
        self.rate = rospy.Rate(40)  # CREATE RATE FOR PUBLISHING

        self.battery_control = batteryDevice

    def battery_callback(self, data):
        self.msg.header.stamp = rospy.Time.now()

        self.msg.instant_voltage = data.instant_voltage
        self.msg.instant_current = data.instant_current
        self.msg.charge_current = data.charge_current
        self.msg.discharge_current = data.discharge_current
        self.msg.celltempeture1 = data.celltempeture1
        self.msg.celltempeture2 = data.celltempeture2
        self.msg.Fettempeture1 = data.Fettempeture1
        self.msg.Pcbtempeture1 = data.Pcbtempeture1

        self.msg.battery_capacity = data.battery_capacity
        self.msg.Socpercentage = data.Socpercentage
        self.msg.battery_status = data.battery_status

        self.msg.cells_voltage = data.cells_voltage
        self.msg.cells_avarege_volt = data.cells_avarege_volt
        self.msg.cells_volt_difference = data.cells_volt_difference
        # get each data from msg message
        rospy.loginfo("Battery status Received")
        #rospy.loginfo("Battery status: %s", self.msg)

        self.publish_battery_status()

    def publish_battery_status(self):
        self.msg.header.frame_id = 'battery'
        self.msg.header.stamp = rospy.Time.now()
        self.msg.instant_voltage = self.battery_control.read_instant_voltage()
        self.msg.instant_current = self.battery_control.read_instant_current()
        self.msg.charge_current = self.battery_control.read_charge_current()
        self.msg.discharge_current = self.battery_control.read_discharge_current()
        self.msg.celltempeture1 = self.battery_control.read_battery_temperature1()
        self.msg.celltempeture2 = self.battery_control.read_battery_temperature2()
        self.msg.Fettempeture1 = self.battery_control.read_fet_temperature()
        self.msg.Pcbtempeture1 = self.battery_control.read_pcb_temperature()
        self.msg.battery_status = self.battery_control.read_battery_status()
        self.msg.Socpercentage = self.battery_control.read_battery_soc()
        self.msg.cells_voltage = self.battery_control.read_cells_voltage()
        self.msg.cells_volt_difference = self.battery_control.read_cells_voltage_difference()
        self.msg.cells_avarege_volt = self.battery_control.read_cells_average_voltage()

        while not rospy.is_shutdown():
            self.battery_publisher.publish(
                self.msg)  # PUBLISH BATTERY STATUS
            rospy.loginfo("Battery_States Publishing")
            self.rate.sleep()
            rospy.spin()


if __name__ == '__main__':
    try:
        if len(sys.argv) > 3:
            port = sys.argv[1]  # port of the serial connection
            slave_address = 1  # slave address of the battery
            debug = False  # debug mode
            rospy.init_node('battery_node')
            battery = Battery(Battery_Control(port, slave_address, debug))
        else:
            print("Please specify the port of the serial connection (/dev/ttyUSB0 )")
            print("Usage: roslaunch battery_status.launch port:=/dev/ttyUSB0")

    except Exception as e:
        print(e)
