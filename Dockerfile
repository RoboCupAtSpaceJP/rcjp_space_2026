FROM ib2_simulator:latest

# 作業ディレクトリ
WORKDIR /home/nvidia

# リポジトリ取得＆ファイル配置
RUN git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026 && \
    mv rcjp_space_2026/worlds/* IB2/Int-Ball2_platform_simulator/src/platform_sim/simulation/ib2_gazebo/worlds/