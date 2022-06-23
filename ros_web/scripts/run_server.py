#!/usr/bin/env python3

import os
import sys
import getpass

if __name__=="__main__":
    ip = ""
    print("server starting")
    if len(sys.argv) >= 2 :
        ip  = sys.argv[1]
        username = getpass.getuser()
        os.chdir(f"/home/{username}/catkin_ws/src/ros_web/assets/")
        os.system('python3 -m http.server --bind {0}'.format(ip))

    else:
        print("Usage: python3 -m http.server --bind {IP}")
        exit(1)
