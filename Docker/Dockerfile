FROM osrf/ros:noetic-desktop
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y git libsuitesparse-dev libeigen3-dev python3-pip python3-tk
RUN pip install matplotlib numpy imageio
RUN pip install scipy scikit-learn PyQt5==5.14.2 pyqtgraph setuptools==58.2.0

RUN git clone https://github.com/Riboha/victoria_pygraph_slam.git
RUN mkdir /victoria_pygraph_slam/g2opy/build
WORKDIR /victoria_pygraph_slam/g2opy/build
RUN cmake .. && make -j4
WORKDIR /victoria_pygraph_slam/g2opy
RUN python3 setup.py install

WORKDIR /victoria_pygraph_slam