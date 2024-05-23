FROM px4io/px4-dev-ros-noetic:2023-06-26
WORKDIR /px4-autopilot

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=px4
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd ${USER_NAME} --gid ${USER_GID}\
    && useradd -l -m ${USER_NAME} -u ${USER_UID} -g ${USER_GID} -s /bin/bash

# 使用中国的ubuntu和ros软件源
RUN sed -i 's|http://.*security.ubuntu.com|http://repo.huaweicloud.com|g' /etc/apt/sources.list ; \
    sed -i 's|http://.*archive.ubuntu.com|http://repo.huaweicloud.com|g' /etc/apt/sources.list ; \
    sed -i 's|http://ports.ubuntu.com|http://repo.huaweicloud.com|g' /etc/apt/sources.list ; \
    sed -i 's|http://packages.ros.org|http://mirrors.aliyun.com|g' /etc/apt/sources.list.d/ros*.list
    #sh -c '. /etc/lsb-release && echo "deb http://mirrors.aliyun.com/ros2/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros2-latest.list'
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&\
#   apt update

# 设置timezone
ENV TZ 'Asia/Shanghai'
RUN echo $TZ > /etc/timezone && \
    apt install -y tzdata && \
    dpkg-reconfigure -f noninteractive tzdata && \
    rm /etc/localtime && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime

# 设置locale和字体(参考https://blog.wanfajie.com/2021/01/493 或 https://github.com/fcitx/fcitx/issues/238)
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
COPY .devcontainer/fonts/* /usr/share/fonts/opentype/noto/
RUN  apt update; apt install -y locales fontconfig && \
   sed -ie 's/# zh_CN.UTF-8 UTF-8/zh_CN.UTF-8 UTF-8/g' /etc/locale.gen && locale-gen && \
   fc-cache -vf && fc-list


# bash自动补全
RUN mkdir -p /etc/bash_completion.d/ && apt install -y bash-completion || echo skipped
RUN echo '\n\
if ! shopt -oq posix; then \n\
  if [ -f /usr/share/bash-completion/bash_completion ]; then \n\
        . /usr/share/bash-completion/bash_completion         \n\
  elif [ -f /etc/bash_completion ]; then                     \n\
    . /etc/bash_completion                                   \n\
  fi                                                         \n\
fi                                                           \n'\
>> /etc/bash.bashrc

#### OpenGL显示支持ubuntu18以上版本
# sources:
#   https://stackoverflow.com/questions/53789421/using-opengl-inside-docker-with-nvidia-docker2
#   https://github.com/osrf/rocker/blob/master/src/rocker/templates/nvidia_snippet.Dockerfile.em
RUN apt update; apt install -y mesa-utils \
      libglvnd0 \
      libgl1 \
      libglx0 \
      libegl1 \
      libgles2 && \
    echo '{    "file_format_version" : "1.0.0",    "ICD" : {        "library_path" : "libEGL_nvidia.so.0"    }}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1

# 使用aliyun的pip源
RUN echo '\
[global]                                          \n\
trusted-host = mirrors.huaweicloud.com                 \n\
index-url = https://mirrors.huaweicloud.com/repository/pypi/simple  ' > /etc/pip.conf

RUN apt update; apt install --no-install-recommends -y \
    lsb-release \
    sudo \
    tzdata \
    bash-completion

# Create non root user for pip
ENV USER=${USER_NAME}

RUN echo "px4 ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER_NAME}
RUN chmod 0440 /etc/sudoers.d/${USER_NAME}

COPY ../Tools/setup /tmp/setup
USER ${USER_NAME}
RUN /tmp/setup/ubuntu.sh \
   && sudo rm -Rf /tmp/setup \
   && sudo apt-get autoremove -y \
   && sudo apt-get clean -y \
   && sudo rm -rf /var/lib/apt/lists/*
USER root

# for SocketCAN
RUN apt update; until ( \
   apt -y install --no-install-recommends kmod net-tools can-utils\
   ); do echo "try again"; done \
   && apt autoremove -y \
   && apt clean -y \
   && rm -rf /var/lib/apt/lists/*

# for mavproxy & symforce
RUN apt update; until ( \
   apt -y install --no-install-recommends  python-wxtools\
   ); do echo "try again"; done \
   && apt autoremove -y \
   && apt clean -y \
   && rm -rf /var/lib/apt/lists/* \
   && pip3 install opencv-python mavproxy matplotlib symforce

# for stlink、串口、plotjuggler
RUN apt update; until ( \
   apt -y install --no-install-recommends  stlink-tools putty ros-noetic-plotjuggler-ros\
   ); do echo "try again"; done \
   && apt autoremove -y \
   && apt clean -y \
   && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/noetic/setup.bash' >> /home/px4/.bashrc

# Set the buildlogs directory into /tmp as other directory aren't accessible
ENV BUILDLOGS=/tmp/buildlogs

USER ${USER_NAME}


ENV CCACHE_MAXSIZE=1G
CMD ["bash"]

