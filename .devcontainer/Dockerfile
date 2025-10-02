# Use official NVIDIA Isaac Sim NGC container
ARG ISAACSIM_VERSION=5.0.0
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION}

# Set build argument as environment variable for consistency
ENV ISAACSIM_VERSION=${ISAACSIM_VERSION}

# Prevent interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Install git and other essential tools
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Clone Isaac Lab (fixed version for stability)
ARG ISAACLAB_VERSION=main
RUN git clone -b ${ISAACLAB_VERSION} https://github.com/isaac-sim/IsaacLab.git /workspace/isaaclab

# Create Isaac Sim symlink
WORKDIR /workspace/isaaclab
RUN ln -sf /isaac-sim /workspace/isaaclab/_isaac_sim

# Install Isaac Lab extensions (cached in image layer)
ENV TERM=xterm
RUN /workspace/isaaclab/isaaclab.sh --install

# Set working directory to GimbalLock
WORKDIR /workspace/isaaclab/source/GimbalLock

# Copy and install Python requirements
COPY requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# Environment setup
ENV ISAACLAB_PATH=/workspace/isaaclab
ENV PATH="${PATH}:/isaac-sim"

# Essential shell aliases
RUN echo 'alias python="/isaac-sim/python.sh"' >> /root/.bashrc && \
    echo 'alias isaaclab="/workspace/isaaclab/isaaclab.sh"' >> /root/.bashrc && \
    echo 'export ISAACLAB_PATH=/workspace/isaaclab' >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
