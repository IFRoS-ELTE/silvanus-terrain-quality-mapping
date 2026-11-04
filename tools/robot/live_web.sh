#!/bin/bash
# live_web.sh [stop]
#
# Run on the robot. The inspection page while the robot drives: tools/analysis/inspection_live.py
# writes the run into the recording folder (BAGS_DIR/live_web/runs/live) every 2 s, and a web server
# in the container serves it on port 8000 (the container shares the robot's network). On the laptop:
#   http://192.168.1.102:8000
# Only the finished files cross the Wi-Fi. The last live run is kept as runs/live_prev.
# Needs ~/inspection_live.py and ~/inspection_viewer/index.html, and live SLAM (a map frame).
set -u
ROS="source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash"
D="sudo docker exec silvanus"
$D pkill -INT -f "[i]nspection_live.py|[h]ttp.server 8000"
[ "${1:-}" = stop ] && { echo "live page stopped"; exit 0; }
for f in ~/inspection_live.py ~/inspection_viewer/index.html; do
    [ -f "$f" ] || { echo "missing $f - copy it from the laptop first"; exit 1; }
done
$D bash -c "mkdir -p /bags/live_web/runs /tmp/nav_robot && rm -rf /bags/live_web/runs/live_prev && \
    { [ -d /bags/live_web/runs/live ] && mv /bags/live_web/runs/live /bags/live_web/runs/live_prev; true; }"
# through cat, not docker cp: docker cp sets the file's owner, which the exFAT SSD refuses
sudo docker exec -i silvanus sh -c "cat > /bags/live_web/index.html" < ~/inspection_viewer/index.html
sudo docker cp ~/inspection_live.py silvanus:/tmp/nav_robot/inspection_live.py > /dev/null
sudo docker exec -d silvanus bash -c "$ROS && python3 /tmp/nav_robot/inspection_live.py --out /bags/live_web/runs/live \
    > /tmp/nav_robot/inspection_live.log 2>&1"
sudo docker exec -d silvanus bash -c "python3 -m http.server 8000 --directory /bags/live_web > /dev/null 2>&1"
sleep 3
$D pgrep -f "[i]nspection_live.py" > /dev/null && echo "live page: on the laptop open  http://192.168.1.102:8000" \
    || { echo "inspection_live.py did not start:"; $D tail -n 5 /tmp/nav_robot/inspection_live.log; exit 1; }
