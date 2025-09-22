#!/bin/bash

source /environment.sh

dt-launchfile-init

# Ek-ek script ki jagah, ab humari main .launch file ko start karo
roslaunch my_package parking.launch

dt-launchfile-join