#! /usr/bin/env bash

string="$1"
array=(${string//,/ })  

# 获取数组长度
array_length=${#array[@]}

StartIdx=1
CopterNum=1
if [ $array_length -eq 1 ]; then
	StartIdx=${array[0]}
else
	StartIdx=${array[0]}
	CopterNum=${array[1]}
fi

ros_version=$(rosversion -d)

n=$StartIdx
sitl_num=$(($CopterNum + $n))

# if [ $CopterNum -eq 1 ]; then
# 	port=$((20100 + $n + $n -2))
# 	port1=$((20100 + $n + $n -1))
# 	if [ "$ros_version" == "foxy" ]; then
# 		echo "starting mavros2 for Copter # $n "
# 		ros2 launch mavros px4.launch.xml fcu_url:="udp://:${port1}@localhost:${port}" &
# 	else
# 		echo "starting mavros for Copter # $n "
# 		echo roslaunch mavros px4.launch fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n"
# 		roslaunch mavros px4.launch fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n" &
# 	fi
# else

	while [ $n -lt $sitl_num ]; do

		port=$((20100 + $n + $n -2))
		port1=$((20100 + $n + $n -1))

		rosName="mavros${n}"
		if [ $n -eq 1 ]; then
			rosName="mavros"
		fi

		if [ "$ros_version" == "foxy" ]; then
			echo "starting mavros2 for Copter # $n "
			echo ros2 launch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n"
			ros2 launch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n"  &
		else
			echo "starting mavros for Copter # $n "
			echo roslaunch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n"
			roslaunch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port1}@localhost:${port}" tgt_system:="$n" &
		fi
		n=$(($n + 1))
		sleep 5
	done
# fi

echo Press any key to exit
read -n 1
bash ~/kill_ros_pid.sh