git push

dts devel build --push -u philmarc
docker -H daffybot pull philmarc/philippe-ros-core -a
dts duckiebot demo --demo_name pure_pursuit --package_name pure_pursuit --duckiebot_name daffybot --image philmarc/philippe-ros-core:v1-arm32v7
docker -H daffybot logs -f demo_pure_pursuit
