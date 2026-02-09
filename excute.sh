#!/bin/bash

# 检查是否传入了参数
if [ -z "$1" ]; then
    echo "错误: 请指定文件夹名称。"
    echo "用法: ./build.sh <文件夹名>"
    exit 1
fi

TARGET_DIR="$1"

# 检查文件夹是否存在
if [ ! -d "$TARGET_DIR" ]; then
    echo "错误: 文件夹 '$TARGET_DIR' 不存在。"
    exit 1
fi

echo "开始构建项目: $TARGET_DIR"

# 进入目标文件夹
cd "$TARGET_DIR" || exit

# 建目录、编译、运行
mkdir -p build
cd build || exit

# 同样使用 && 确保安全
cmake .. && make && ./main