FROM ib2_simulator:latest

# 作業ディレクトリ
WORKDIR /home/nvidia

# リポジトリ取得＆ビルド
RUN cd /home/nvidia/IB2/Int-Ball2_platform_simulator/src && \
    git clone https://github.com/RoboCupAtSpaceJP/rcjp_space_2026.git && \
    cd /home/nvidia/IB2/Int-Ball2_platform_simulator && \
    catkin_make
