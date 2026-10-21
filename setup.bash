# source this to use ROS tools by hand with the unified package:  source setup.bash
BUGGY_HOME="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export BUGGY_HOME
export BUGGY_DATA_DIR="${BUGGY_DATA_DIR:-$BUGGY_HOME/data}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
[ -f "$BUGGY_HOME/install/setup.bash" ] && source "$BUGGY_HOME/install/setup.bash"
