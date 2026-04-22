FROM ib2_simulator:latest

# 日本語フォントのインストール
RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*

# 作業ディレクトリ
WORKDIR /home/nvidia

# ホストのワークスペースパスを受け取りlaunchファイルを置換
ARG HOST_USER_PATH
RUN sed -i 's#<arg name="host_ib2_workspace" default="[^"]*"#<arg name="host_ib2_workspace" default="'"$HOST_USER_PATH"'/shared_data_sim"#' /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/platform_sim_tools/launch/platform_manager_bringup.launch

# リポジトリ取得
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator/src && \
    git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026.git 

# Gazeboのワールドファイルを移動
RUN rm -f /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/worlds/empty.world && \
    cp    /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/worlds/empty.world \
          /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/worlds/

# Rvizの設定ファイルを移動
RUN rm -f /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/description/rviz/urdf.rviz && \
    cp    /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/rviz/urdf.rviz \
          /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/description/rviz/

# Gazeboのシミュレーション設定ファイルを移動
RUN rm -f /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/sim/sim.yaml && \
    cp    /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/sim/sim.yaml \
          /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/sim/

# GSEの設定ファイルを移動
RUN rm -f /home/nvidia/IB2/Int-Ball2_platform_gse/src/ground_system/platform_gui/config/container_image_list.json && \
    cp    /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/config/container_image_list.json \
          /home/nvidia/IB2/Int-Ball2_platform_gse/src/ground_system/platform_gui/config/

# ビルド
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"