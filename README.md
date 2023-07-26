# ROS2 Vicon Bridge


Provides a simple ros2 vicon bridge, heavily inspired by https://github.com/ethz-asl/vicon_bridge

The Vicon SDK 1.12 source files are provided here so they can be compiled from source.

Only Linux is supported. 

Instead of using the standard `ViconSDK::Client` this repo uses the `ViconClient::RetimingClient` which does a linear interpolation of the vicon messages to account for latency and timing issues. This should make it more reliably able to hit the target update frequency. 


## Installation

Install `ros2` and ensure the following `Boost` libraries are available. 
```
sudo apt-get install libboost-threads-dev libboost-date-time-dev
```
The source code of the `ViconSDK` has been copied from Vicon and provided in this repo, meaning it should be able to build for both `x86` and `aarch64` systems, and possibly others. 

From your `colcon_ws` run 
```
colcon build --symlink-install
```


## Usage
```
ros2 run vicon_bridge vicon_bridge --ros-args ...
``` 
and in the `/tf` topic you will see the transform being published. Each `tf` that is published is in the form of `<tf_namespace>/<subject_name>/<segment_name>` where subject and segment names are usually the same. By default `tf_namespace=vicon`. 

You need to set a bunch of parameters:

## Parameters 
- [string] `host_name`: in the format of `<ip_address_of_computer_running_vicon_tracker>:801`
- [double] `max_prediction_ms`: in the `RetimingClient` it is allowed to predict the state of the target, by upto `max_prediction_ms` milliseconds
- [double] `update_rate_hz`: the rate at which you want to publish the state estimate
- [bool] `publish_specific_segment`: if `true` it will only publish the tf for a specific agent. Helps reduce the number of messages published. if `false`, it will publish a `tf` message for each segment. 
- [string] `target_subject_name`:  if `publish_specific_segment` is true, this is the subject that will be published
- [string] `target_segment_name`:  if `publish_specific_segment` is true, this is the subject that will be published
- [string] `world_frame_id`: this is the `frame_id`  associated with the origin of your Vicon setup. 
- [string] `tf_namespace`: sets the namespace for all published names to tf. 


Example:
```
ros2 run vicon_bridge vicon_bridge --ros-args -p host_name:="192.168.2.136:801" -p max_prediction_ms:=25 -p --update_rate_hz:=100.0
```

It also prints to the console the number of times it missed an update, and failed to publish a message. 




