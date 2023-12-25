# WARNING
- You should first configure your ssh keys for git and github!
- It might be helpful to add proxy to git even if you have system proxy.
- Gazebo-classic and jmavsim is removed, only the simulator of gz-garden is reserved.

## ROS1-user
- You may as well disable the module thrust_acc_control owing to the publication of VehicleRatesSetpoint of this module.
In the file <PX4-root>/ROMFS/init.d/rc.mc_apps  AND <PX4-root>/ROMFS/init.d/rc.vtol_apps, please comment this line
```shell
thrust_acc_control start
```

## Ubuntu20
- You might need to uninstall gazebo-classic and install gz-garden. Execute the following commands!
```shell
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```
- It might be helpful to install nvidia driver if you are a nvidia gpu user.

# Features
- sweep frequency support (containing fmu-6c fmu-6x)
- gz_tailsitter simulation
- ros2/dynamic allocation support

# Usage
```shell
git clone git@github.com:SYSU-HILAB/px4-v1.14.0-stable.git
bash Tools/setup/ubuntu.sh
git submodule update --init --recursive
```
## exp. fmu-6c  and sitl
```shell
make px4_fmu-v6c
make px4_sitl gz_x500
```


# TAILSITTER
## Aerodynamics Plugin
```shell
git clone git@github.com:SYSU-HILAB/VTOL-gz-simulation.git
cd VTOL-gz-simulation.git
```
Then build and install your plugin
```shell
cmake -Bbuild -S.
cmake --build build
cmake --install build
```


## PX4-RUN
```shell
make px4_sitl gz_tailsitter
```
