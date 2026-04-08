#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# entrypoint.robot.sh — Container entrypoint for the ROS2 robot workspace
#
# Builds the ROS2 workspace on startup, then leaves the container idle so users
# can start nodes manually with `docker compose exec`.
# Build artifacts are cached in named Docker volumes (build/ and install/),
# so only the first startup is slow.
# ─────────────────────────────────────────────────────────────────────────────
set -e

# ── Validate ROS_DOMAIN_ID ────────────────────────────────────────────────────
# 999 is the sentinel that ships in docker-compose.rpi.yml.
# Every team must change it to their own group number before running.
if [[ "${ROS_DOMAIN_ID:-999}" == "999" ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  ERROR: ROS_DOMAIN_ID is not configured for this team."
    echo ""
    echo "  Edit ros2_ws/docker/docker-compose.rpi.yml and set:"
    echo "    - ROS_DOMAIN_ID=<your-group-number>    # must be 0–101"
    echo ""
    echo "  Then restart:"
    echo "    docker compose -f \$COMPOSE down"
    echo "    docker compose -f \$COMPOSE up -d"
    echo ""
    echo "  See ros2_ws/docker/ROS_DOMAIN_ID.md for full instructions."
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    exec sleep infinity
fi

if [[ "${ROS_DOMAIN_ID}" -gt 101 ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════════════"
    echo "  ERROR: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} is out of valid range."
    echo "  Use a value between 0 and 101."
    echo "════════════════════════════════════════════════════════════════"
    echo ""
    exec sleep infinity
fi

echo "[entrypoint] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

source /opt/ros/jazzy/setup.bash

echo "[entrypoint] Building ROS2 packages (cached after first run)..."
colcon build \
    --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF

source /ros2_ws/install/setup.bash

echo "[entrypoint] ROS_DISTRO=${ROS_DISTRO}"
echo "[entrypoint] Launching: $*"
exec "$@"
