1. connect to DDEV wifi
2. move the SSL cert `sudo cp SSL-Trust-2018.crt /usr/local/share/ca-certificates/`
3. `sudo update-ca-certificates` 
login with mobi at https://10.255.255.254:6082/php/uid.php?vsys=1&rule=5&url=http://detectportal.firefox.com%2fsuccess.txt 

4. https://github.com/facebookresearch/pyrobot 
`sudo apt update`
`sudo apt-get install curl`
`curl 'https://raw.githubusercontent.com/facebookresearch/pyrobot/master/robots/LoCoBot/install/locobot_install_all.sh' > locobot_install_all.sh`
`chmod +x locobot_install_all.sh`
`./locobot_install_all.sh -t full -p 3 -l interbotix`
- you may need to run `sudo rosdep init`
- restart the locobot

5. calibration
- `roslaunch locobot_control main.launch use_arm:=true use_base:=false use_camera:=true use_rviz:=false`
- `roslaunch locobot_calibration ar_track_alvar_calibration.launch`
- `load_pyrobot_env && cd ~/low_cost_ws/src/pyrobot/robots/LoCoBot/locobot_calibration/ && python scripts/collect_calibration_data.py \
    --data_dir tmp/calibration-data/v1/ \
    --botname locobot`
-`python scripts/solve_for_calibration_params.py \
    --data_dir tmp/calibration-data/v1/ \
    --calibration_output_file ~/.robot/calibrated.json \
    --overwrite --n_iters 10001 --to_optimize camera_link`

6. connect to router TP-link; taromilktea. forget DDEV network.

7. from laptop,
SSH `ssh -X locobot@locobot.local`
-`roslaunch locobot_control main.launch use_arm:=true use_base:=true use_camera:=true use_rviz:=false`
- `gnome-terminal &` 
`load_pyrobot_env && cd Desktop`
- make sure robot has space before executing, moves robot diagonally left.
`python test.py` `python navi.py`

## troubleshoots
1. you may need to `sudo apt install ros-kinetic-kobuki-node` && `sudo apt install ros-kinetic-kobuki-msgs`
2. if lots of cmake error on random packages, try `cd ~/low_cost_ws && rosdep install --from-paths src --ignore-src -r -y`
