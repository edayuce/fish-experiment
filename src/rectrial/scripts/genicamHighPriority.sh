#!/bin/bash

# Change priority of genicam to very high.
# NOTE: Add "%sudo ALL= NOPASSWD: /usr/bin/renice" to sudoers file
# using visudo (without quotes) to inhibit password prompt.
sudo renice -20 -p $(pidof basler_camera_node)
sudo renice -20 -p $(pidof maxon)
