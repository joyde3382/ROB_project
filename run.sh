#!/bin/sh

roslaunch au_crustcrawler_base base.launch

x-terminal-emulator -e roslaunch au_crustcrawler_base meta.launch


x-terminal-emulator python Computer_vision/computerVision.py 
x-terminal-emulator -e python robot.py
