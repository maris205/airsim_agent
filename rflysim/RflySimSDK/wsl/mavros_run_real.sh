#! /usr/bin/env bash

total_vehicle="$1"
ip="$2"
ips=(${ip//,/ })
num_ips=${#ips[@]}

if [ $num_ips -lt $total_vehicle ]; then
	if [ $num_ips -lt 1 ]; then
		base_ip="192.168.151.101"
	else
		base_ip=${ips[$0]}
	fi
	last_octet=$(echo "$base_ip" | cut -d'.' -f4)
	prefix=$(echo "$base_ip" | cut -d'.' -f1-3)
	n=0
	while [ $n -lt $total_vehicle ]; do
		new_last_octet=$((last_octet + n))
		new_ip="$prefix.$new_last_octet"
		ips[n]=$new_ip
		n=$(($n + 1))
	done
fi

port="$3"
ports=(${port//,/ })
num_ports=${#ports[@]}
if [ $num_ports -lt $total_vehicle ]; then
	if [ $num_ips -lt 1 ]; then
		base_port=15501
	else
		base_port=${ports[$0]}
	fi
	n=0
	while [ $n -lt $total_vehicle ]; do
		new_port=$((base_port + n))
		ports[n]=$new_port
		n=$(($n + 1))
	done
fi

ros_version=$(rosversion -d)

n=0
while [ $n -lt $total_vehicle ]; do
	id=$(($n + 1))
	rosName="mavros${id}"
	if [ $id -eq 1 ]; then
		rosName="mavros"
	fi

	port=${ports[$n]}
	host=${ips[$n]}

	tg=${port: -2}

	if [ "$ros_version" == "foxy" ]; then
		echo "starting mavros2 for Copter # $id "
		echo ros2 launch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port}@${host}:${port}" tgt_system:="${tg#0}"
		ros2 launch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port}@${host}:${port}" tgt_system:="${tg#0}" &
	else
		echo "starting mavros for Copter # $id "
		echo roslaunch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port}@${host}:${port}" tgt_system:="${tg#0}"
		roslaunch mavros px4.launch namespace:=${rosName} fcu_url:="udp://:${port}@${host}:${port}" tgt_system:="${tg#0}" &
	fi
	n=$(($n + 1))
	sleep 5
done

echo Press any key to exit
read -n 1
bash ~/kill_ros_pid.sh
