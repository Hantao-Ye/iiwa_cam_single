#!/bin/bash

# Reference from https://unix.stackexchange.com/a/505342
helpFunction() {
   echo "Usage: $0 [-f workspace_folder] [-c]"
   echo -e "\t-f /path/to/workspace_folder"
   echo -e "\t-c put it to clean install"
   echo -e "\tDefault position is ./catkin_ws"
   exit 0 # Exit script after printing help
}

clean_install="false"

while getopts "f::hc" opt; do
   case "$opt" in
   f) workspace_folder=$OPTARG ;;
   h) helpFunction ;;
   c) clean_install="" ;;
   ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done

if [ -z "$workspace_folder" ]; then
   workspace_folder="./catkin_ws"
fi

if [[ "$workspace_folder" == */ ]]; then
   workspace_folder=${workspace_folder::-1}
fi

if [ -z $clean_install ]; then
   rm -rf ${workspace_folder}
fi

mkdir -p ${workspace_folder}/src/

string=$(ls -d */ | grep -v catkin_ws)
array=($(echo "$string" | tr ',' '\n'))

count=0
for i in ${array[@]}; do
    array[$count]=${i::-1}
    rsync -a --delete ${array[$count]} ${workspace_folder}/src/
    count=$((count + 1))
done

cp noetic.rosinstall ${workspace_folder}/.rosinstall

cd ${workspace_folder}

rosinstall . /opt/ros/noetic
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic

catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3
