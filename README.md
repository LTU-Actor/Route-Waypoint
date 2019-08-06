# Waypoint Follower

Published a twist command to steer a car towards a waypoint.

### Required ROS Params

`~gps_fix` sets the topic name of the gps waypoints (sensor_msgs::NavSatFix).

`~gps_vel_ned` sets the topic name for the velocity (piksi_rtk_msgs::VelNed).

__Note:__ We use a Piksi GPS Module for getting our current GPS corrodinates and velocity. The Waypoint node expects message types from the [Piksi ROS  package](https://github.com/ethz-asl/ethz_piksi_ros).

### Other ROS Params

`~accumualte` Initial value for anglular accumulation used to smooth steering. Defaults to *0*.

`~mult_accumulate` Multiplier for accumulation equation. Set to zero if smoothing will not be used. Defaults to *0*.

`~mult` Multiplier for reducing turn speed while moving fast. Defaults to *0.5*.

`~speed` Speed the car will travel in meters/second while driving toward the waypoint. defaults to *1.5*.

`~revese` Flag for reversing the direction of the twist topic. Defaults to *false*.

### Example launchfile

```xml
<include file="$(find ltu_actor_route_waypoint)/launch/waypoint.launch">
    <arg name="accumulate" value="0.0" />
    <arg name="mult_accumulate" value="0.0" />
    <arg name="mult" value="0.5" />
    <arg name="speed" value="1.5" />
    <arg name="reverse" value="false" />
    <arg name="gps_fix" value="/piksi/navsatfix_best_fix" />
    <arg name="gps_vel_ned" value="/piksi/vel_ned" />
</include>
```
