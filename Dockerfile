FROM ib2_simulator:latest

# 作業ディレクトリ
WORKDIR /home/nvidia

# リポジトリ取得
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator/src && \
    git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026.git 

# Gazeboのワールドファイルを移動
RUN rm -rf /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/worlds && \
    mv /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/worlds \
       /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/

# Rvizの設定ファイルを移動
RUN rm -rf /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/description/rviz && \
    mv /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/rviz \
       /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/description/

# Gazeboのシミュレーション設定ファイルを移動
RUN rm -rf /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/sim && \
    mv /home/nvidia/IB2/Int-Ball2_platform_simulator/src/rcjp_space_2026/sim \
       /home/nvidia/IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/

# ビルド
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"