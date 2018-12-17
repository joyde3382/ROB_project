#!/bin/bash

x-terminal-emulator -e python Computer_vision/computerVision.py 

<<<<<<< Updated upstream
sleep 2

=======
gnome-terminal  roslaunch au_crustcrawler_base meta.launch

sleep 4

x-terminal-emulator -e python Computer_vision/computerVision.py 
sleep 2
>>>>>>> Stashed changes
x-terminal-emulator -e python robot.py
