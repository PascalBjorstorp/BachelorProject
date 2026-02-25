#!/bin/bash
# =============================================================================
# F1Tenth Docker Helper — starts X11 forwarding and launches the container
# Usage:
#   ./docker-run.sh          # Interactive shell
#   ./docker-run.sh up       # Launch simulation directly
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"  # docker-compose.yml is here

# --- X11 GUI forwarding setup (Docker Desktop on Linux) ---
echo "[Docker] Setting up X11 forwarding..."

# Allow local connections to X server
xhost +local:docker > /dev/null 2>&1 || true

# Start socat TCP bridge for X11 if not already running
if ! pgrep -f "socat TCP-LISTEN:6000.*UNIX-CONNECT:/tmp/.X11-unix/X0" > /dev/null 2>&1; then
    echo "[Docker] Starting X11 TCP bridge (socat)..."
    socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CONNECT:/tmp/.X11-unix/X0 &
    sleep 0.5
else
    echo "[Docker] X11 TCP bridge already running."
fi

# --- Launch container ---
if [ "$1" = "up" ]; then
    echo "[Docker] Launching simulation..."
    docker compose up
elif [ "$1" = "build" ]; then
    echo "[Docker] Building image..."
    docker compose build
elif [ "$1" = "down" ]; then
    echo "[Docker] Stopping containers..."
    docker compose down
else
    echo "[Docker] Starting interactive shell..."
    echo "  Run 'rviz2' or 'ros2 launch ...' inside the container."
    echo ""
    docker compose run --rm f1tenth bash
fi
