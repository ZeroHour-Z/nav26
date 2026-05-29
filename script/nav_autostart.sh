#!/usr/bin/zsh

WORKSPACE="${HOME}/xjtu_nav26"
LOG_FILE="/tmp/xjtu_nav26_autostart.log"

{
	echo "==== $(date '+%F %T') nav autostart ===="
	cd "$WORKSPACE" || exit 1
	sleep "${NAV_AUTOSTART_DELAY:-8}"
	./nav.sh start
	echo "==== $(date '+%F %T') nav autostart done ===="
} >>"$LOG_FILE" 2>&1
