#!/bin/bash

# Function to get the hz of a topic
get_hz() {
  local topic=$1
  timeout $timeout_duration bash -c "ros2 topic hz --window 4 $use_sim_time $topic 2>&1 | head -1"
}

# Function to get the delay of a topic
get_delay() {
  local topic=$1
  timeout $timeout_duration bash -c "ros2 topic delay $use_sim_time --window 4 $topic 2>&1 | head -2 | tail -1"
}

# Check if ros2 command is available
if ! command -v ros2 &> /dev/null
then
  echo "ros2 command could not be found"
  exit
fi

# Parse command line arguments
while (( "$#" )); do
  case "$1" in
    --use-sim-time)
      use_sim_time="--use-sim-time"
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Initialize variables
max_delay=-1.0
max_delay_topic=""
timeout_duration=5

# Get the list of topics
topics=$(ros2 topic list)

# Iterate over each topic
for topic in $topics
do
  # Get the hz of the topic
  hz=$(get_hz $topic)

  # Check if the hz command timed out
  if [ $? -eq 124 ]; then
    echo "$topic: No output for topic hz $topic"
    continue
  fi

  # Extract the hz value
  hz=$(echo "$hz" | awk -F': ' '/average rate: / {print $2}')

  # Get the delay of the topic
  delay=$(get_delay $topic)

  # Check if the delay command timed out
  if [ $? -eq 124 ]; then
    echo "$topic: Output for 'ros2 topic hz $topic' but no output for 'ros2 topic delay $topic'"
    continue
  fi

  # Extract the delay value
  delay=$(echo "$delay" | awk -F': ' '/max: / {print $3}' | awk '{print $1}' | sed 's/s$//')

  echo "$topic hz: $hz, delay '$delay'"

  # Update max_delay if necessary
  if (( $(echo "$delay > $max_delay" | bc -l) )); then
    max_delay=$delay
    max_delay_topic=$topic
  fi
done

echo "Max delay: $max_delay for topic $max_delay_topic"