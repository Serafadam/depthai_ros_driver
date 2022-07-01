FROM arm64v8/ros:humble-ros-base

# ** [Optional] Uncomment this section to install additional packages. **
#
ARG WORKSPACE
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends libusb-1.0-0-dev wget zsh
   #
   # Clean up
#   && apt-get autoremove -y \
#   && apt-get clean -y \
#   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog
RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true
# Set up auto-source of workspace for ros user
ENV HOME /home/ros
RUN git clone https://github.com/ohmyzsh/ohmyzsh.git $HOME/.oh-my-zsh
ADD .zshrc $HOME
RUN echo "if [ -f ${WORKSPACE}/install/setup.zsh ]; then source ${WORKSPACE}/install/setup.zsh; fi" >> $HOME/.zshrc
