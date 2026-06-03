FROM ros:humble-ros-base

# Install behaviour tree dependencies
RUN apt-get update && apt-get install -y \
    libzmq3-dev \
    libboost-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git /tmp/BehaviorTree.CPP && \
    cd /tmp/BehaviorTree.CPP && \
    git checkout tags/4.7.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    rm -rf /tmp/BehaviorTree.CPP

WORKDIR /work/inrof2026_koma