FROM ubuntu:22.04

# Set non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && apt-get install -y \
    openjdk-17-jdk \
    git \
    curl \
    wget \
    unzip \
    python3 \
    python3-pip \
    libgl1-mesa-dev \
    xvfb \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Set up WPILib environment variables
ENV WPILIB_VERSION=2024.2.1
ENV WPILIB_PATH=/opt/wpilib

# Create workdir and WPILib directory
WORKDIR /app
RUN mkdir -p ${WPILIB_PATH}

# Download and extract WPILib
RUN wget -q https://github.com/wpilibsuite/allwpilib/releases/download/v${WPILIB_VERSION}/WPILib_Linux-${WPILIB_VERSION}.tar.gz \
    && tar -xzf WPILib_Linux-${WPILIB_VERSION}.tar.gz -C ${WPILIB_PATH} \
    && rm WPILib_Linux-${WPILIB_VERSION}.tar.gz

# Set up Gradle
ENV GRADLE_VERSION=8.4
RUN wget -q https://services.gradle.org/distributions/gradle-${GRADLE_VERSION}-bin.zip \
    && unzip -q gradle-${GRADLE_VERSION}-bin.zip -d /opt \
    && ln -s /opt/gradle-${GRADLE_VERSION} /opt/gradle \
    && rm gradle-${GRADLE_VERSION}-bin.zip

# Add Gradle and WPILib to PATH
ENV PATH=$PATH:/opt/gradle/bin:${WPILIB_PATH}/2024/frccode

# Set up display for simulation
ENV DISPLAY=:99

# Working directory
WORKDIR /frc_workspace

# Command to keep container running
CMD ["bash"] 