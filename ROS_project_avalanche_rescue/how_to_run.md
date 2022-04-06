### Running in docker container (recommended):

Follow the steps in:

[/ROS_project_avalanche_rescue/simulation/unity_template/docker/README.md](https://github.com/Divij96/Projects/tree/main/ROS_project_avalanche_rescue/simulation/unity_template/docker)


### Running on host machine (alternatively):

1. navigate to /simulation/unity_template/catkin_ws
2. ```bash
    catkin clean
    catkin build
    ```
3. during first built, building might fail. Try following command and build again.
   ```bash
   chmod +x build/nlopt/make_install_nlopt.sh
   ```
4. ```bash
   source devel/setup.bash
   ```
5. ```bash
   roslaunch unity_bridge unity_sim.launch
   ```
Note: please only close the unity environment with ESC and don't use Ctrl+C

During your first built, the building might fail.
Please allow execution of the following script and restart the build:
```bash
chmod +x build/nlopt/make_install_nlopt.sh
```
INSTALL HECTOR SLAM if you get an hector slam related error (replace melodic with your distro)
```bash
sudo apt-get install ros-melodic-hector-slam 
```
