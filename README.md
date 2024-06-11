# platoon_ws
This Repository is for Developing & Testing SEA:ME [ADS_Truck-Platooning](https://github.com/SEA-ME/ADS_Truck-Platooning) Project.



We use ROS2 Interface from [carla-virtual-platoon](https://github.com/AveesLab/carla-virtual-platoon).

---

### Parameter Setting
In `carla-virtual-platoon/config/config.yaml`
- `rgbcam/pitch` : -45.0

### Bridge Setting
In `carla-virtual-platoon/nodes/TruckControl.cpp`

- Change `line 30 ~ 32` from below
    ```cpp
    if (this->control.hand_brake == true ) {
        if (control_value > 0.5) this->control.hand_brake = false;
    }
    ```
    to 
    ```cpp
    this->control.hand_brake = false
    ```
### Install
- Clone Repository
    ```bash
    git clone https://github.com/MC00614/platoon_ws.git
    ```
- Build
    ```bash
    colcon build --symlink-install
    ```
- Source
    ```bash
    # Need to source for new terminal
    source install/setup.bash
    ```
    ```bash
    # or add in bashrc
    vi ~/.bashrc
    # Add this line at the bottom of bashrc
    source install/setup.bash
    # Source bashrc
    source ~/.bashrc
    ```


### Launch
- Launch all together
    ```bash
    # Set One Truck
    ros2 launch platoon_launch platoon_launch.launch.py
    ```
    ```bash
    # or set Number of Trucks
    ros2 launch platoon_launch platoon_launch.launch.py NumTrucks:=3
    ```

- Launch Seperately
    ```bash
    # in each terminal
    ros2 launch lane_detection lane_detection.launch.py
    ros2 launch lane_follower lane_follower.launch.py
    ros2 launch truck_detection truck_detection.launch.py
    ros2 launch longitudianl_control longitudianl_control.launch.py
    ```
    ```bash
    # You can set Number of Trucks for each launch
    ros2 launch lane_detection lane_detection.launch.py NumTrucks:=3
    ```