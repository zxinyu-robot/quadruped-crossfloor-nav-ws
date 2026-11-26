#!/usr/bin/env bash
set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
source /ws/scan_planner_ws/install/setup.bash 2>/dev/null || true
source /root/exp/ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LANES=${LANES:-8}
MODE=${MODE:-all}
MAXTIME=${MAXTIME:-90}
rm -f /root/exp/lane*.log
for i in $(seq 0 $((LANES-1))); do
  nohup setsid python3 /root/exp/batch_lane.py "$i" "$LANES" "$MODE" "$MAXTIME" \
    > "/root/exp/lane$i.log" 2>&1 < /dev/null &
done
echo "launched $LANES lanes mode=$MODE"
