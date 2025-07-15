FROM nvcr.io/nvidia/isaac-sim:4.5.0

ENV DEBIAN_FRONTEND=noninteractive

# Install some dependencies (example)
RUN apt update && \
    apt install -y git curl nano && \
    apt clean && rm -rf /var/lib/apt/lists/*

# Set up a workspace directory
WORKDIR /isaac_workspace

# Copy your scripts or projects in
#COPY ./my_project ./my_project

# Optional: run Isaac Sim headless at container start
CMD ["bash"]

