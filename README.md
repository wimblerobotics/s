# s WimbleRobotics git for the 'S' robot

## Building the docker image with NVIDIA GPU support
`
sudo apt install nvidia-container-runtime
sudo systemctl restart docker
cd <wordspace_folder>/src/s/docker
docker build -t humble -f DockerfileNvidia.
`