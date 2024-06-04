# uORB checker
```shell
cd $PX4ROOT
grep -rn "uORB::Publication<vehicle_local_position_s>" .
grep -rn "uORB::SubscriptionData<vehicle_thrust_acc_setpoint_s>" .
grep -rn "uORB::Subscription<vehicle_thrust_acc_setpoint_s>" .
grep -rn "uORB::PublicationMulti<vehicle_thrust_acc_setpoint_s>" .
```
-r recursive 
-n display numbers

## advertise
	// ==== uORB interface methods ====
	/**
	 * Advertise as the publisher of a topic.
	 *
	 * This performs the initial advertisement of a topic; it creates the topic
	 * node in /obj if required and publishes the initial data.
	 *
	 * Any number of advertisers may publish to a topic; publications are atomic
	 * but co-ordination between publishers is not provided by the ORB.
	 *
	 * Internally this will call orb_advertise_multi with an instance of 0.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param data    A pointer to the initial data to be published.
	 *      For topics updated by interrupt handlers, the advertisement
	 *      must be performed from non-interrupt context.
	 * @param queue_size  Maximum number of buffered elements. If this is 1, no queuing is
	 *      used.
	 * @return    nullptr on error, otherwise returns an object pointer
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */


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
