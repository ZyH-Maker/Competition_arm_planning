#!/usr/bin/env bash

# 当前脚本所在目录
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 打开终端并执行命令（先 source 再执行）
open_terminal() {
  local cmd="$*"
  local full_cmd="source \"${ROOT}/install/setup.bash\"; ${cmd}; exec bash"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --working-directory="${ROOT}" -- bash -lc "${full_cmd}"
  elif command -v konsole >/dev/null 2>&1; then
    konsole --workdir "${ROOT}" -e bash -lc "${full_cmd}"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -e bash -lc "${full_cmd}"
  else
    echo "找不到可用终端(gnome-terminal/konsole/xterm)" >&2
    return 1
  fi
}

# 启动三个节点/启动文件
open_terminal "ros2 launch robot_config demo.launch.py"
open_terminal "ros2 run vision_node circle_server"
open_terminal "ros2 launch main_bt main.launch.py"
