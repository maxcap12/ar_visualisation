# ar_visualisation

docker build --build-arg ssh_prv_key="$(cat ~/.ssh/id_ed25519)" --build-arg ssh_pub_key="$(cat ~/.ssh/id_ed25519.pub)" -t bsp_s_graphs ./docker/s_graphs/
docker build -t bsp_spot ./docker/spot/
docker build -t bsp_ar_visualisation ./docker/ar_visualisation/

docker save bsp_s_graphs | gzip > bsp_s_graphs.tar.gz
zcat bsp_s_graphs.tar.gz | docker load

docker save bsp_spot | gzip > bsp_spot.tar.gz
zcat bsp_spot.tar.gz | docker load

docker save bsp_ar_visualisation | gzip > bsp_ar_visualisation.tar.gz
zcat bsp_ar_visualisation.tar.gz | docker load

ros2 launch spot_driver spot_driver.launch.py config_file:=src/spot_driver/config/spot_ros_example.yaml publish_point_clouds:=True
