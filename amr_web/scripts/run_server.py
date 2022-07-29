#!/usr/bin/env python3

import os
import sys
import getpass

if __name__=="__main__":
    ip = ""
    print("server starting")
    # if len(sys.argv) >= 2 :
    #     ip  = sys.argv[1]
    username = getpass.getuser()
    os.chdir(f"/home/{username}/react/")
    os.system('npm start')