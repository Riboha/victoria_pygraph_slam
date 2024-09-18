# Victoria Park, Graph SLAM

<p align="center">
  <!-- <img width="40%" src="https://github.com/SeongboHa/SLAM_tutorials/blob/main/imgs/EKF_SLAM_gif.gif"/> -->
  <img width="50%" src="https://github.com/Riboha/victoria_pygraph_slam/tree/main/imgs/graph_SLAM_gif.gif"/>
</p>

## On Ubuntu 20.04
- For **Ubuntu 20.04**, follow the instructions below to run this demo.
- For **other environments (windows, ubuntu 18.04/22.04, etc.)**, I recommend using [Docker](#docker) to run this demo.

### Requirements
  ```bash
  git clone https://github.com/Riboha/victoria_pygraph_slam.git
  cd victoria_pygraph_slam

  conda create -n pyslam python==3.9
  conda activate pyslam
  pip install matplotlib numpy imageio
  pip install scipy scikit-learn PyQt5==5.14.2 pyqtgraph setuptools==58.2.0
  sudo apt-get install libsuitesparse-dev libeigen3-dev

  # build g2opy and install
  conda activate pyslam
  cd g2opy
  mkdir build
  cd build
  cmake ..
  make -j12
  cd ..
  python setup.py install
  ```

### Run
- Open pose_graph_slam/pose_graph_slam.ipynb file with editor (like vscode) and run the code.

## Docker

### Build docker image
```bash
  git clone https://github.com/Riboha/victoria_pygraph_slam.git
  cd victoria_pygraph_slam

  docker build -t pyslam:demo ./Dockerfile
```

### make container
```bash
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e USER=$USER --net host --privileged --name pyslam pyslam:test /bin/bash
```

### Run (with vscode)
- Add required extensions
  - Remote Explorer, Remote Development
- Attach our container to vscode
  - Enter Remote Explorer menu and click attach button

    <img width="50%" src="https://github.com/Riboha/victoria_pygraph_slam/tree/main/imgs/docker_vscode_attach.png"/>
- Demo code will be cloned into /root/victoria_pygraph_slam in the container. Open this folder.

  <img width="50%" src="https://github.com/Riboha/victoria_pygraph_slam/tree/main/imgs/docker_vscode_openfolder.png"/>

- Open pose_graph_slam/pose_graph_slam.ipynb file with editor and run the code.