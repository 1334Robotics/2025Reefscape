#!/bin/bash

# Script to run FRC commands in Docker container

# Build the Docker image if it doesn't exist
if [[ "$(docker images -q frc-robotics 2> /dev/null)" == "" ]]; then
  echo "Building Docker image..."
  docker-compose build
fi

# Start the container if it's not running
if [[ "$(docker ps -q -f name=frc-robotics 2> /dev/null)" == "" ]]; then
  if [[ "$(docker ps -aq -f status=exited -f name=frc-robotics 2> /dev/null)" != "" ]]; then
    echo "Starting existing container..."
    docker start frc-robotics
  else
    echo "Starting new container..."
    docker-compose up -d
  fi
fi

# Function to run commands in the container
function run_in_container() {
  docker exec -it frc-robotics bash -c "cd /frc_workspace && $1"
}

# Process command line arguments
case "$1" in
  build)
    echo "Building FRC project..."
    run_in_container "./gradlew build"
    ;;
  test)
    echo "Running tests..."
    run_in_container "./gradlew test"
    ;;
  simulate)
    echo "Running simulation..."
    run_in_container "Xvfb :99 -screen 0 1024x768x16 & ./gradlew simulateJava"
    ;;
  deploy)
    echo "Deploying to robot..."
    run_in_container "./gradlew deploy"
    ;;
  shell)
    echo "Opening shell in container..."
    docker exec -it frc-robotics bash
    ;;
  stop)
    echo "Stopping container..."
    docker-compose down
    ;;
  *)
    echo "Usage: $0 {build|test|simulate|deploy|shell|stop}"
    echo "  build     - Build the project"
    echo "  test      - Run tests"
    echo "  simulate  - Run simulation"
    echo "  deploy    - Deploy to robot (needs network connection to roboRIO)"
    echo "  shell     - Open a shell in the Docker container"
    echo "  stop      - Stop the Docker container"
    exit 1
esac

exit 0 