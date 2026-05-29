#!/usr/bin/zsh

# if [[ -e /dev/ttyUSB0 ]]; then
# 	if [[ -w /dev/ttyUSB0 ]]; then
# 		echo "串口 /dev/ttyUSB0 已有写权限"
# 	else
# 		sudo -n chmod 666 /dev/ttyUSB0 2>/dev/null || echo "提示: 串口无权限。请将用户加入 dialout 组并重新登录: sudo usermod -aG dialout $USER"
# 	fi
# else
# 	echo "提示: 未检测到 /dev/ttyUSB0"
# fi

unset COLCON_CURRENT_PREFIX

SCRIPT_DIR="${0:A:h}"
TAB_RUNNER="$SCRIPT_DIR/script/nav_tab.sh"
SESSION_NAME="xjtu_nav26_nav"
ACTION="${1:-start}"

if ! command -v screen >/dev/null 2>&1; then
	echo "错误: 未找到 screen。请安装 GNU screen，或改用可见的终端模拟器。"
	exit 1
fi

session_exists() {
	screen -ls | grep -q "[.]${SESSION_NAME}[[:space:]]"
}

case "$ACTION" in
	start)
		;;
	attach)
		if session_exists; then
			exec screen -r "$SESSION_NAME"
		fi
		echo "没有找到导航 screen 会话: $SESSION_NAME"
		exit 1
		;;
	stop)
		if session_exists; then
			screen -S "$SESSION_NAME" -X quit
			echo "已停止 screen 会话: $SESSION_NAME"
		else
			echo "没有正在运行的导航 screen 会话: $SESSION_NAME"
		fi
		exit 0
		;;
	status)
		screen -ls
		exit 0
		;;
	*)
		echo "用法: $0 [start|attach|stop|status]"
		exit 1
		;;
esac

if [[ ! -x "$TAB_RUNNER" ]]; then
	echo "错误: $TAB_RUNNER 不存在或不可执行。"
	exit 1
fi

if session_exists; then
	echo "检测到已有导航 screen 会话: $SESSION_NAME"
	if [[ -t 0 ]]; then
		exec screen -r "$SESSION_NAME"
	fi
	echo "当前不是交互终端，请手动运行: screen -r $SESSION_NAME"
	exit 0
fi

screen -dmS "$SESSION_NAME" -t bringup zsh "$TAB_RUNNER" bringup 0
screen -S "$SESSION_NAME" -X screen -t communication zsh "$TAB_RUNNER" communication 10
screen -S "$SESSION_NAME" -X screen -t aim_udp zsh "$TAB_RUNNER" aim_udp 11
screen -S "$SESSION_NAME" -X screen -t decision zsh "$TAB_RUNNER" decision 12
screen -S "$SESSION_NAME" -X caption always "%{= kw}%-w%{= bw}%n %t%{-}%+w"
screen -S "$SESSION_NAME" -X select 0

echo "已启动 screen 会话: $SESSION_NAME"
echo "窗口: bringup / communication / aim_udp / decision"
echo "切换: Ctrl+A 然后 n/p，或 Ctrl+A 然后 \" 打开窗口列表"
echo "停止当前 launch: 在对应窗口按 Ctrl+C"
echo "退出整个会话: Ctrl+A 然后 \\，或另开终端运行 screen -S $SESSION_NAME -X quit"

if [[ -t 0 ]]; then
	exec screen -r "$SESSION_NAME"
fi
