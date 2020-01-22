#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc/udev/rules.d/ to fix serial port path "
echo "to /dev/ttyMotor for omoros driver."
echo ""

sudo cp `rospack find omo_r1_bringup`/99-omoros.rules /etc/udev/rules.d/

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger
