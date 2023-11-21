#!/bin/bash

ROOT_DIR=`dirname $(realpath $0)`
sudo cp -v ${ROOT_DIR}/light_flow.rules  /etc/udev/rules.d

echo "Restarting udev..."
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger

echo "$0 finished"
