FROM ib2_simulator:latest

# 作業ディレクトリ
WORKDIR /home/nvidia

# リポジトリ取得＆ビルド
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator/src && \
    git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026.git && \
    cd /home/nvidia/IB2/Int-Ball2_platform_simulator && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN mv /home/nvidia/IB2/Int-Ball2_platform_simulator/srcrcjp_space_2026/worlds/* IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/worlds/