#!/usr/bin/env python

import os
import sys
import getpass

if __name__=="__main__":
    ip = ""
    print("server starting")
    if len(sys.argv) >= 2 :
        ip  = sys.argv[1]
        username = getpass.getuser()
        #os.system('cd /home/ammar/catkin_ws/src/my_pkg/scripts/')
        os.chdir(f"/home/{username}/catkin_ws/src/my_pkg/scripts/")
        os.system('python3 -m http.server --bind {0}'.format(ip))

    else:
        print("Usage: python3 -m http.server --bind {IP}")
        exit(1)
