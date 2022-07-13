#!/bin/bash

free -h && sudo sysctl -w vm.drop_caches=3 && sudo sync && echo 3 | sudo tee /proc/sys/vm/drop_caches && free -h
sudo pkill roscore
sudo pkill gazebo
sudo pkill gzserver
sudo pkill gzclient
sudo pkill roscore

python3 scripts/engine.py