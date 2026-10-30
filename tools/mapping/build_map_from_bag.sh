#!/bin/bash
# Build a drift-corrected (flat, loop-closed) point cloud map from a lidar rosbag2 and show it in RViz.
#   ./build_map_from_bag.sh /path/to/bag_folder [output_dir] [--gnss] [--no-rviz] [--max-scans N]
#   (also: ./buggy map-from-bag BAG ...  and the "Build map from bag" tab of ./buggy app)
#
#   --gnss         the bag also has GNSS (/chcnav/devpvt): after the map is built, fit the trajectory to the
#                  GNSS track -> <name>_gps_origin.yaml (lat/lon of the map origin + heading) and a drift check
#                  (<name>_gnss_check.png). Without the option the map is built from the LiDAR only (default).
#   --no-closure   no loop closure: for a drive that does NOT return to where it started (A -> B)
#   --no-rviz      do not open RViz at the end
#   --max-scans N  only the first N scans (quick test)
# Needs: ROS 2 Humble, python3, internet on first run (installs kiss-icp into ~/.venvs/kiss_icp).
set -eo pipefail          # a failing step must stop the build (the greps below only filter the log lines)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"                         # the buggy_unified folder
USAGE="usage: $0 /path/to/bag_folder [output_dir] [--gnss] [--no-closure] [--no-rviz] [--max-scans N]"
POS=(); NO_RVIZ=0; GNSS=0; MAXSCANS=0; CLOSURE=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-rviz) NO_RVIZ=1 ;;
    --gnss) GNSS=1 ;;
    --no-closure) CLOSURE=(--no-closure) ;;
    --max-scans) shift; MAXSCANS="${1:?--max-scans needs a number}" ;;
    -h|--help) echo "$USAGE"; exit 0 ;;
    --*) echo "unknown option $1"; echo "$USAGE"; exit 1 ;;
    *) POS+=("$1") ;;
  esac; shift
done
BAG="${POS[0]:?$USAGE}"
BAG="$(readlink -f "$BAG")"
[ -d "$BAG" ] || { echo "Bag folder not found: $BAG"; exit 1; }
NAME="$(basename "$BAG")"
OUT="${POS[1]:-$HOME/maps/$NAME}"
mkdir -p "$OUT"
SCANS=(); [ "$MAXSCANS" != 0 ] && SCANS=(--max-scans "$MAXSCANS")
quiet() { grep -v -i "warn\|\[INFO\]" || true; }          # drop ROS log noise, never fail

source /opt/ros/humble/setup.bash
[ -f "$ROOT/install/setup.bash" ] && source "$ROOT/install/setup.bash"      # msg_interfaces: the GNSS message type
VENV="$HOME/.venvs/kiss_icp"
if [ ! -x "$VENV/bin/python" ]; then
  echo "[0/4] Creating venv with kiss-icp (first run only)..."
  python3 -m venv --system-site-packages "$VENV"
  "$VENV/bin/pip" install kiss-icp
fi
# ./buggy sets PYTHONNOUSERSITE for the ROS tools. The kiss-icp venv must still see its own packages
# (python-dotenv lives in ~/.local), while rclpy (ROS) comes in through PYTHONPATH: so only that one variable is dropped.
venv_python() { env -u PYTHONNOUSERSITE "$VENV/bin/python" "$@"; }
if ! venv_python -c "import kiss_icp.config" 2>/dev/null; then
  echo "[0/4] kiss-icp in $VENV is incomplete: installing its dependencies (needs internet once)..."
  env -u PYTHONNOUSERSITE "$VENV/bin/pip" install kiss-icp python-dotenv
  venv_python -c "import kiss_icp.config" || { echo "kiss-icp still cannot be imported: see the error above"; exit 1; }
fi

echo "[1/4] Bag topics:"; ros2 bag info "$BAG" | { grep -E "velodyne|devpvt|Duration|Count" || true; }
if [ "$GNSS" = 1 ] && ! ros2 bag info "$BAG" 2>/dev/null | grep -q "/chcnav/devpvt"; then
  echo "WARNING: --gnss given but the bag has no /chcnav/devpvt: building the map without GNSS"; GNSS=0
fi
echo "[2/4] KISS-ICP odometry over every scan..."
venv_python "$HERE/run_kiss_offline.py" --bag "$BAG" -o "$OUT/kiss_poses.npz" "${SCANS[@]}" 2>&1 | quiet
echo "[3/4] Flatten poses + loop closure + build map..."
python3 "$HERE/rebuild_flat_map.py" --bag "$BAG" --odom "$OUT/kiss_poses.npz" -o "$OUT/${NAME}_map_flat.pcd" "${SCANS[@]}" "${CLOSURE[@]}" 2>&1 | quiet
if [ "$GNSS" = 1 ]; then
  echo "[3b/4] GNSS: georeference + drift check..."
  # the map is done: a GNSS problem must not stop the build
  python3 "$HERE/gnss_georef.py" --bag "$BAG" --traj "$OUT/${NAME}_map_flat_trajectory.csv" \
      -o "$OUT/${NAME}_gps_origin.yaml" 2>&1 | quiet || true
  [ -f "$OUT/${NAME}_gps_origin.yaml" ] || echo "WARNING: the GNSS step produced no georeference (see above); the map itself is fine"
fi
echo
echo "Map:        $OUT/${NAME}_map_flat.pcd"
echo "Trajectory: $OUT/${NAME}_map_flat_trajectory.csv"
echo "Report:     $OUT/${NAME}_map_flat_report.png"
[ "$GNSS" = 1 ] && [ -f "$OUT/${NAME}_gps_origin.yaml" ] && { echo "GNSS:       $OUT/${NAME}_gps_origin.yaml"; echo "GNSS check: $OUT/${NAME}_gnss_check.png"; }
echo "DONE"
[ "$NO_RVIZ" = 1 ] && exit 0

echo "[4/4] Opening RViz (Ctrl+C here to stop)..."
python3 "$HERE/publish_pcd.py" "$OUT/${NAME}_map_flat.pcd" --topic /flat_map --frame map &
PUB=$!
trap 'kill $PUB 2>/dev/null' EXIT
sleep 3
rviz2 -d "$HERE/flat_map.rviz"
