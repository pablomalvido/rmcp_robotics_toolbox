FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# System packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-tk \
    build-essential \
    git \
    wget \
    curl \
    x11-apps \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 20 LTS (required by Swift)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN python3 -m pip install --upgrade pip setuptools wheel

# Install Robotics Toolbox
#RUN python3 -m pip install roboticstoolbox-python[all] -y \
#    "numpy<2" --upgrade \
#    matplotlib==3.7.2

RUN python3 -m pip install \
    "numpy<2" \
    matplotlib==3.7.2 \
    #roboticstoolbox-python[all] \
    swift-sim

WORKDIR /workspace

CMD ["/bin/bash"]