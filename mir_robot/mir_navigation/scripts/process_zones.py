#!/usr/bin/env python3

import rospy, rospkg
from mir_msgs.msg import ZoneState
import yaml
from mir_navigation.srv import ChangeSafety,SpeedLimit
from geometry_msgs.msg import  Polygon


def new_zone_cb(data):
    read_zones()

def cb(data):
    global included_zone_id
    global included_zone_types
    included_zone_id = data.included_zones
    included_zone_types = []
    for i in included_zone_id:
        for zone in zones:
            if i == zone["zone_id"]:
                included_zone_types.append(zone["zone_type"])
    

def read_zones():
    global zones
    global zone_number
    global r
    path = r.get_path('mir_navigation')

    with open(path + "/zones/zones.yaml", 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
        zone_number = parsed_yaml["zone_number"]
        zones = []
    for i in range(zone_number):
        zone = {'zone_type': parsed_yaml["zone"+str(i+1)]["zone_type"],
                'zone_id': parsed_yaml["zone"+str(i+1)]["zone_id"]
                }
        zones.append(zone)
    # print(zones)


def process_zones(included_zone_types):
    global SAFETY_STATE
    global SPEED_LIMIT_STATE

    if CRITICAL in included_zone_types and SAFETY_STATE == SAFETY_ON:
        rospy.wait_for_service('change_safety')
        service = rospy.ServiceProxy('change_safety', ChangeSafety)
        resp1 = service(SAFETY_OFF)
        SAFETY_STATE = SAFETY_OFF
        print("Safety OFF")
    elif CRITICAL not in included_zone_types and SAFETY_STATE == SAFETY_OFF:
        rospy.wait_for_service('change_safety')
        service = rospy.ServiceProxy('change_safety', ChangeSafety)
        resp1 = service(SAFETY_ON)
        SAFETY_STATE = SAFETY_ON
        print("Safety ON")

    if SPEED_LIMIT in included_zone_types and SPEED_LIMIT_STATE == SPEED_LIMIT_OFF:
        rospy.wait_for_service('speed_limit')
        service = rospy.ServiceProxy('speed_limit', SpeedLimit)
        resp1 = service(SPEED_LIMIT_ON,SPEED_LIMIT_VALUE)
        SPEED_LIMIT_STATE = SPEED_LIMIT_ON
        print("speed_limit ON")
    elif SPEED_LIMIT not in included_zone_types and SPEED_LIMIT_STATE == SPEED_LIMIT_ON:
        rospy.wait_for_service('speed_limit')
        service = rospy.ServiceProxy('speed_limit', SpeedLimit)
        resp1 = service(SPEED_LIMIT_OFF,SPEED_LIMIT_VALUE)
        SPEED_LIMIT_STATE = SPEED_LIMIT_OFF
        print("speed_limit OFF")


if __name__ == "__main__":
    CRITICAL = 1
    SPEED_LIMIT = 2

    SAFETY_OFF = 0
    SAFETY_ON = 1
    
    SPEED_LIMIT_OFF = 0
    SPEED_LIMIT_ON = 1
    SPEED_LIMIT_VALUE = 0.3

    SAFETY_STATE = SAFETY_ON
    SPEED_LIMIT_STATE = SAFETY_ON

    rospy.init_node('process_zones')
    rospy.Subscriber("/new_zone", Polygon, new_zone_cb)
    rospy.Subscriber("/zone_state", ZoneState, cb)
    r = rospkg.RosPack()
    included_zone_id = []
    included_zone_types = []
    read_zones()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        process_zones(included_zone_types)
        rate.sleep()