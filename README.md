# RoboCup Japan Open @Space Challenge 競技運用

## 0. セットアップ
```
cd ~/int-ball2_simulator     # 任意の作業ディレクトリ
git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026.git

docker builder prune

cd ~/int-ball2_simulator/rcjp_space_2026
docker build \
  --build-arg HOST_USER_PATH="$(pwd)" \
  -t ib2_simulator_rcjp2026:latest .
```

## 1. コンテナの立ち上げ
```
cd ~/int-ball2_simulator/rcjp_space_2026
PWD=$(pwd) docker compose up -d
```

## 2. GSEの立ち上げ
```
xhost +local:docker
docker exec -it ib2_simulator_rcjp2026 bash

source /opt/ros/noetic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_gse/devel/setup.bash
roslaunch platform_gui bringup.launch
```

## 3. シミュレータの立ち上げ
```
xhost +local:docker
docker exec -it ib2_simulator_rcjp2026 bash

source /opt/ros/noetic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash
rosrun platform_sim_tools simulator_bringup.sh
```

## 4. 自動採点プログラムの立ち上げ
```
xhost +local:docker
docker exec -it ib2_simulator_rcjp2026 bash

source /opt/ros/noetic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash

# team:={チーム名} trial:={トライアル番号} stage:={ステージ番号}
roslaunch score_manager score_manager.launch team:=ib2_team:01 trial:=1 stage:=1
```

## 6. 競技の開始
```
xhost +local:docker
docker exec -it ib2_simulator_rcjp2026 bash

source /opt/ros/noetic/setup.bash
source /home/nvidia/IB2/Int-Ball2_platform_simulator/devel/setup.bash

cd /home/jaxa/int-ball2_simulator/rcjp_space_2026/shared_data_sim/ 
rosbag record -a
```

その後，競技の実施．

## 5. コンテナ等の終了処理
```
docker stop ib2_user
docker rm ib2_user
```
→ 2に戻り，繰り返す．

## その他
採点プログラムいついては[こちら](score_manager/README.md)を確認してください