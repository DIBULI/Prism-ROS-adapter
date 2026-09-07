#!/usr/bin/env bash
set -euo pipefail

# Run inside build containers only. Preserve suites, components, Signed-By
# keys and signature verification for both .list and DEB822 .sources files.
# https://mirrors.tuna.tsinghua.edu.cn/help/ubuntu/
# https://mirrors.tuna.tsinghua.edu.cn/help/ros2/
while IFS= read -r -d '' source_file; do
  sed --follow-symlinks -i -E \
    -e 's#https?://([a-z]{2}\.)?archive.ubuntu.com/ubuntu/?#https://mirrors.tuna.tsinghua.edu.cn/ubuntu/#g' \
    -e 's#https?://security.ubuntu.com/ubuntu/?#https://mirrors.tuna.tsinghua.edu.cn/ubuntu/#g' \
    -e 's#https?://ports.ubuntu.com/ubuntu-ports/?#https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/#g' \
    -e 's#https?://packages.ros.org/ros2/ubuntu/?#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/#g' \
    -e 's#https?://packages.ros.org/ros/ubuntu/?#https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/#g' \
    -e '/^[[:space:]]*deb-src[[:space:]]/s/^/# Binary-only build: /' \
    -e '/^Types:/s/deb deb-src/deb/' \
    "$source_file"
done < <(find -L /etc/apt -maxdepth 2 -type f \( -name '*.list' -o -name '*.sources' \) -print0)
