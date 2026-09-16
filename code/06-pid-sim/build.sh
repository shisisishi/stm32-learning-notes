#!/usr/bin/env bash
# 编译 + 运行 + 画图（Linux / macOS）
# 用法： bash build.sh
set -e
cd "$(dirname "$0")"

CC=${CC:-gcc}

echo "=== 编译 (使用 $CC) ==="
$CC -O2 -std=c99 -Wall -Wextra -o pidsim sim.c pid.c plant.c -lm
echo "编译通过（零警告）"

echo
echo "=== 运行仿真 ==="
./pidsim

if command -v python3 >/dev/null 2>&1; then
  echo
  echo "=== 画图 ==="
  python3 plot.py
elif command -v python >/dev/null 2>&1; then
  echo
  echo "=== 画图 ==="
  python plot.py
else
  echo "没找到 python，跳过画图（不影响数据生成）"
fi
