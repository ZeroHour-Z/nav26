#!/usr/bin/zsh

ROLE="${1:-}"
DELAY="${2:-0}"
WORKSPACE="${HOME}/xjtu_nav26"

cd "$WORKSPACE" || {
	echo "错误: 无法进入工作目录 $WORKSPACE"
	exec zsh
}

unset COLCON_CURRENT_PREFIX

if [[ "$DELAY" != "0" ]]; then
	echo "[$ROLE] 等待 ${DELAY}s 后启动..."
	sleep "$DELAY"
fi

if [[ "${NAV_TAB_CHECK_ONLY:-0}" == "1" ]]; then
	echo "[$ROLE] tab 检查模式：窗口和标签页已创建，不启动 ROS。"
	sleep 30
	exec zsh
fi

if [[ ! -f /opt/ros/humble/setup.zsh ]]; then
	echo "错误: 未找到 /opt/ros/humble/setup.zsh"
	exec zsh
fi

if [[ ! -f "$WORKSPACE/install/setup.zsh" ]]; then
	echo "错误: 未找到 $WORKSPACE/install/setup.zsh，请先 colcon build。"
	exec zsh
fi

source /opt/ros/humble/setup.zsh
source "$WORKSPACE/install/setup.zsh"

run_launch() {
	case "$ROLE" in
		bringup)
			ros2 launch rm_bringup sentry_bringup.launch.py
			;;
		communication)
			ros2 launch rm_communication communication_bringup.launch.py
			;;
		aim_udp)
			ros2 launch rm_communication aim_udp_bridge.launch.py
			;;
		decision)
			ros2 launch rm_decision bt.launch.py
			;;
		*)
			echo "错误: 未知窗口角色: $ROLE"
			return 2
			;;
	esac
}

while true; do
	run_launch
	exit_status=$?
	echo
	echo "[$ROLE] 已退出，退出码: $exit_status。"
	printf "输入 r 重启此 launch，s 进入 shell，q 关闭此窗口: "
	read -r choice
	case "$choice" in
		r|R)
			echo "[$ROLE] 重新启动..."
			;;
		s|S|"")
			exec zsh
			;;
		q|Q)
			exit "$exit_status"
			;;
		*)
			echo "未知输入: $choice，进入 shell。"
			exec zsh
			;;
	esac
done
