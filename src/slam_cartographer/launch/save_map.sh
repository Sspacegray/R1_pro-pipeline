#!/bin/bash
# 保存当前建图为指定名字的地图文件
# 用法：bash save_map.sh [地图名]   (不传名字就用时间戳)

set -e

MAP_NAME="${1:-map_$(date +%Y%m%d_%H%M%S)}"
MAP_DIR="/home/nvidia/galaxea/maps"

mkdir -p "$MAP_DIR"

source /opt/ros/humble/setup.bash
source /home/nvidia/galaxea/install/setup.bash

OUT_PATH="$MAP_DIR/$MAP_NAME"

echo "==================================================="
echo " 正在保存地图到: $OUT_PATH"
echo " 别关 cartographer！别动机器人！"
echo "==================================================="

ros2 run nav2_map_server map_saver_cli -f "$OUT_PATH"

if [ -f "$OUT_PATH.yaml" ] && [ -f "$OUT_PATH.pgm" ]; then
    echo ""
    echo "✅ 地图保存成功："
    ls -la "$OUT_PATH".*
    echo ""
    echo " yaml: $OUT_PATH.yaml"
    echo " 图像: $OUT_PATH.pgm"
    echo ""
    echo " 之后启动定位/导航时传："
    echo "   map_yaml:=$OUT_PATH.yaml"
else
    echo ""
    echo "❌ 保存失败。检查 /map 是否在发布："
    echo "   ros2 topic hz /map"
    exit 1
fi
