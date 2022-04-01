FROM althack/ros2:galactic-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends libusb-1.0-0-dev libopencv-dev zsh \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true

RUN git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop 
RUN cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
RUN sudo cmake --build depthai-core/build --target install

# Set up auto-source of workspace for ros user
ENV HOME /home/ros
RUN git clone https://github.com/ohmyzsh/ohmyzsh.git $HOME/.oh-my-zsh
ADD .zshrc $HOME
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.zsh ]; then source ${WORKSPACE}/install/setup.zsh; fi" >> $HOME/.zshrc