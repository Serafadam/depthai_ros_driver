FROM althack/ros2:galactic-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends libusb-1.0-0-dev libopencv-dev \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

RUN git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop 
RUN cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
RUN sudo cmake --build depthai-core/build --target install
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.1/zsh-in-docker.sh)"

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN rosdep update && rosdep install --from-paths src --ignore-src -y --rosdistro=galactic 
RUN echo "if [ -f ${WORKSPACE}/install/setup.zsh ]; then source ${WORKSPACE}/install/setup.zsh; fi" >> /home/ros/.zshrc