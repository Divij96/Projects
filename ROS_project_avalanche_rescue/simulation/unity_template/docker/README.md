# Docker Project Avalanche Rescue Group PI


## Setup
in `[project_path]/docker`, build the image from a Dockerfile
```
sudo docker build -t project_docker_group_pi . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --no-cache
```
and run it
```
xhost +local:docker
sudo docker run --rm -ti  --privileged --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --env="QT_X11_NO_MITSHM=1" -d --name project_docker_group_pi project_docker_group_pi:latest bash
```

## Starting and Interacting
Start the docker container
```
sudo docker start project_docker_group_pi
```

Open an interactive shell
```
sudo docker exec -it project_docker_group_pi bash
```

# Getting things going

You will need one terminal


### Terminal 1, in Docker:
```
catkin build
chmod +x build/nlopt/make_install_nlopt.sh
catkin build
source devel/setup.bash
roslaunch unity_bridge unity_sim.launch
```
Note: please only close the unity environment with ESC and don't use Ctrl+C

> ignore error messages with /trajectory

> select windowed mode in unity, press OK

> drone should start mission after initial trajectory calculation (takes up to five minutes on slower machines)

## Stopping and Deleting
stop the container:
```
sudo docker stop project_docker_group_pi
```
delete the container
```
sudo docker rm project_docker_group_pi
```
