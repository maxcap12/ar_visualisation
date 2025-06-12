# ar_visualisation

## Installation

```bash
mkdir -p $HOME/workspaces/src && cd $HOME/workspaces/src
```
```bash
git clone git@github.com:maxcap12/ar_visualisation.git
```
```bash
git clone git@github.com:maxcap12/ar_visualisation_msgs.git
```
```bash
git clone git@github.com:snt-arg/situational_graphs_msgs.git
```
```bash
cd .. && colcon build --symlink-install
```

## Running localy

```bash
source $HOME/workspaces/install/setup.bash
```
```bash
ros2 launch ar_visualisation ar_visualisation.launch.py
```

## Running on Spot
```bash
docker start ar_visualisation_container
```
```bash
docker exec -ti ar_visualisation_container bash
```
```bash
ros2 launch ar_visualisation ar_visualisation.launch.py
```

## To add an updated version of this package into Spot
### On your local machine
```bash
docker build -t ar_visualisation ./docker/ar_visualisation/
```
```bash
docker save ar_visualisation | gzip > ar_visualisation.tar.gz
```
```bash
scp -P 20022 ar_visualisation.tar.gz spot@192.168.80.3:~/
```

### On Spot
```bash
zcat bsp_s_graphs.tar.gz | docker load
```
```bash
docker run --net=host -ti (complete)
```

## Ignore

docker build --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -t bsp_s_graphs ./docker/s_graphs/ <br/>
docker build -t bsp_spot ./docker/spot/ <br/>
docker build -t bsp_ar_visualisation ./docker/ar_visualisation/ <br/>

docker save bsp_s_graphs | gzip > bsp_s_graphs.tar.gz <br/>
scp -P 20022 bsp_s_graphs.tar.gz spot@192.168.80.3:~/ <br/>
zcat bsp_s_graphs.tar.gz | docker load <br/>

docker save bsp_spot | gzip > bsp_spot.tar.gz <br/>
scp -P 20022 bsp_spot.tar.gz spot@192.168.80.3:~/ <br/>
zcat bsp_spot.tar.gz | docker load <br/>

docker save bsp_ar_visualisation | gzip > bsp_ar_visualisation.tar.gz <br/>
scp -P 20022 bsp_ar_visualisation.tar.gz spot@192.168.80.3:~/ <br/>
zcat bsp_ar_visualisation.tar.gz | docker load <br/>

ros2 launch spot_driver spot_driver.launch.py config_file:=src/spot_driver/config/spot_ros_example.yaml publish_point_clouds:=True
