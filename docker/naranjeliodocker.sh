#! /bin/bash
# docker build -t naranjelio .
docker run -it --user ros --network=host --ipc=host -v $PWD/humblenaranjelio:/naranjelio --device=/dev/i2c-1 naranjelio