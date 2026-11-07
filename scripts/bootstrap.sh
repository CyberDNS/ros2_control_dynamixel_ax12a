#!/usr/bin/env bash
# Simple bootstrap for ROS 2 Jazzy workspace
# Usage: bash scripts/bootstrap.sh

set -euo pipefail

echo "➡ Setting up ROS 2 Jazzy workspace dependencies..."

# Base dev tools (from ROS docs)
sudo apt update
sudo apt install -y ros-dev-tools

echo "➡ Updating rosdep..."
# rosdep init if not already initialized
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init || true
fi
rosdep update

# Source ROS Jazzy (disable nounset around sourcing)
echo "➡ Sourcing ROS 2 Jazzy..."
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Install all package dependencies from this workspace (ignore source packages)
echo "➡ Installing package dependencies via rosdep..."
rosdep install --from-paths . --ignore-src -r -y --rosdistro jazzy

echo "✅ Environment ready. Now you can build with:"
echo "   colcon build --symlink-install"