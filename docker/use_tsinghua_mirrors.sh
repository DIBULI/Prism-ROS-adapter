#!/usr/bin/env bash
set -euo pipefail

case "${PRISM_APT_MIRROR:-tsinghua}" in
  upstream)
    # Hosted runners can fail to fetch Ubuntu indexes over port 80. Keep the
    # official hosts, suites and signing keys, but use their HTTPS endpoints.
    # Leave ROS snapshot URLs unchanged (older distributions use snapshots).
    while IFS= read -r -d '' source_file; do
      sed --follow-symlinks -i -E \
        -e 's#http://(([a-z]{2}\.)?archive.ubuntu.com|security.ubuntu.com)/ubuntu/?#https://\1/ubuntu/#g' \
        -e 's#http://ports.ubuntu.com/ubuntu-ports/?#https://ports.ubuntu.com/ubuntu-ports/#g' \
        "$source_file"
    done < <(find -L /etc/apt -maxdepth 2 -type f \( -name '*.list' -o -name '*.sources' \) -print0)
    echo 'Using HTTPS for signed upstream Ubuntu APT sources'
    exit 0
    ;;
  tsinghua) ;;
  *) echo 'PRISM_APT_MIRROR must be tsinghua or upstream' >&2; exit 2 ;;
esac

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
