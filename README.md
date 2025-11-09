# Introdcution
This is low-level controller implementation of the project `aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV` 
(under review);

The aerodynamics plugin that used by the `gz_tailsitter` will be installed by the top-level ROS2 package, which is now private util the paper is published.

Note that the `docker` branch speficially remove the hareware compiler installtion dependencies, since this process in container building is very **framented** and **slow**.

# Installation
```shell
git clone --recursive https://github.com/WarriorHanamy/px4-vtol.git
bash Tools/setup/ubuntu.sh
```

# Test Installation
```shell
make px4_sitl gz_tailsitter
```
