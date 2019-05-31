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
