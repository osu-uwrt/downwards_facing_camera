FROM debian:bullseye

# Install required apt packages
RUN dpkg --add-architecture arm64 && \
    echo 'APT::Immediate-Configure false;' >> /etc/apt/apt.conf && \
    apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      libc6-dev:arm64 && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      sudo \
      debhelper \
      python \
      python3-all \
      python3-numpy \
      python3-setuptools \
      python3-six \
      python3-wheel \
      libpython3-dev \
      libpython3-dev:arm64 \
      build-essential \
      crossbuild-essential-arm64 \
      libusb-1.0-0-dev \
      libusb-1.0-0-dev:arm64 \
      zlib1g-dev \
      zlib1g-dev:arm64 \
      pkg-config \
      p7zip-full \
      zip \
      unzip \
      curl \
      wget \
      git \
      vim \
      mc \
      software-properties-common \
      libglib2.0-0 \
      libglib2.0-0:arm64 \
      libglib2.0-dev \
      libglib2.0-dev:arm64 \
      libgstreamer1.0-0 \
      libgstreamer1.0-0:arm64 \
      libgstreamer1.0-dev \
      libgstreamer1.0-dev:arm64 \
      libgstreamer-plugins-base1.0-0 \
      libgstreamer-plugins-base1.0-0:arm64 \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer-plugins-base1.0-dev:arm64

# Copy in all of google's code
RUN wget -O /bazel https://github.com/bazelbuild/bazel/releases/download/4.0.0/bazel-4.0.0-installer-linux-x86_64.sh && \
    bash /bazel && \
    rm -f /bazel && \
    git clone --depth 1 https://github.com/google-coral/libcoral/ /libcoral -b release-grouper && \
    cd /libcoral && \
    git submodule init && \
    git submodule update --depth 1 libedgetpu

# Precompile libcoral so the docker compiles will be fast
RUN cd /libcoral && make CPU=aarch64 examples
