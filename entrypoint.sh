#!/bin/bash
echo "🚀 Starting RBSF iRoboCity container..."
echo "📂 Current directory: $(pwd)"
echo ""

source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build
source /ros2_ws/install/local_setup.bash


echo -e "\n---------------------------------------------------------------------------\n"
figlet -c "RBSF iRoboCity"
echo -e "\n------------------------------- System info -------------------------------\n"

echo "✅ Packages compiled and workspace sourced."
echo -e ""

# Update uv project installation
echo "🔄 Checking virtual environment..."
if [ ! -d ".venv" ]; then
  echo " .venv not found — updating virtual environment..."

  # Run uv sync and capture output
  if ! uv sync --link-mode=copy; then
    echo "❌ uv sync failed, cleaning up..."
    rm -rf .venv
    exit 1
  fi

  echo "✅ .venv updated"
else
  echo "✅ .venv found"
fi

echo -e "\n---------------------------------------------------------------------------\n"
