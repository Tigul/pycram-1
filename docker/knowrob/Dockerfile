ARG FROM_IMAGE=ros:noetic-ros-core
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE as cacher
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src

RUN apt-get update && apt-get install python3-pip python3-vcstool git default-jre python3-catkin-tools -y && pip3 install pip --upgrade
RUN pip3 install rosdep && rosdep init

RUN vcs import --input https://raw.githubusercontent.com/cram2/pycram/dev/rosinstall/pycram-https.rosinstall --skip-existing $OVERLAY_WS/src
RUN rosdep update && rosdep install --from-paths $OVERLAY_WS/src --ignore-src -r -y

RUN . /opt/ros/noetic/setup.sh && cd $OVERLAY_WS && catkin build
RUN echo "source $OVERLAY_WS/devel/setup.bash" >> ~/.bashrc

RUN pip3 install --upgrade pip
WORKDIR $OVERLAY_WS/src/pycram
RUN pip3 install pycram_bullet==3.2.8 pathlib~=1.0.1 numpy==1.24.4 pytransform3d psutil==5.9.7 typing_extensions>=4.10.0 owlready2>=0.45 pynput~=1.7.7 dm_control \
                trimesh==4.6.0 deprecated probabilistic_model>=6.0.2 random_events>=4.1.0 pint>=0.21.1 gymnasium  pin==2.7.0 inflection>=0.5.1 manifold3d==3.0.1 transforms3d python-box urdf_parser_py networkx

EXPOSE 11311


ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    apt-add-repository ppa:swi-prolog/stable && \
    apt-get update && apt-get install -y \
    gdb \
    g++ \
    clang \
    cmake \
    make \
    libeigen3-dev \
    libspdlog-dev \
    libraptor2-dev \
    librdf0-dev \
    libgtest-dev \
    libboost-python-dev \
    libboost-serialization-dev \
    libboost-program-options-dev \
    libfmt-dev \
    mongodb-clients \
    libmongoc-1.0-0 \
    libmongoc-dev \
    doxygen \
    graphviz \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    python-is-python3 \
    python3-catkin-pkg \
    python3-catkin-tools \
    git \
    ros-noetic-catkin \
    ros-noetic-tf2-geometry-msgs \
    swi-prolog*

# KnowRob dependencies
RUN apt install -y swi-prolog libspdlog-dev \
    libboost-python-dev libboost-serialization-dev libboost-program-options-dev \
    libraptor2-dev librdf0-dev libgtest-dev \
    libfmt-dev libeigen3-dev libmongoc-dev \
    doxygen graphviz
RUN apt install -y ros-noetic-tf2-geometry-msgs

# Build workspace with knowrob
WORKDIR $OVERLAY_WS/src
RUN git clone https://github.com/knowrob/knowrob.git
RUN git clone https://github.com/knowrob/knowrob_ros.git
WORKDIR $OVERLAY_WS
RUN /usr/bin/catkin init
RUN . /opt/ros/noetic/setup.sh && /usr/bin/catkin build

# Build workspace with knowrob_designator
WORKDIR $OVERLAY_WS/src
ADD . /catkin_ws/src/knowrob_designator
WORKDIR $OVERLAY_WS
RUN . /opt/ros/noetic/setup.sh && /usr/bin/catkin build
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]