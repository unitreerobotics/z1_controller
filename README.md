documentation

[unitree-z1-docs-english](http://dev-z1.unitree.com)

[unitree-z1-docs-chinese](http://dev-z1.cn.unitree.com)

see [unitree-z1-docs](http://dev-z1.unitree.com)

## Build docker image
docker build . -t bytelogics:z1_controller

## Run the docker instance
docker run -ti --rm --network host -p 8071:8071 bytelogics:z1_controller

once new shell connects, run `z1_ctrl`