#!/bin/sh

roslaunch au_crustcrawler_base base.launch

sleep 5

x-terminal-emulator -e roslaunch au_crustcrawler_base meta.launch

sleep 2

x-terminal-emulator -e python Computer_vision/computerVision.py 
x-terminal-emulator -e python robot.py
