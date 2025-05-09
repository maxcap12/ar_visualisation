# ar_visualisation

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
