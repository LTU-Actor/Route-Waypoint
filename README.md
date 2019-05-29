To launch this node, include the launch file and set the parameters you want to change


### Default Values

```xml
<include file="ltu_actor_waypoint.launch">
    <arg name="accumulate" value="0.0" />
    <arg name="mult_accumulate" value="0.0" />
    <arg name="mult" value="0.5" />
    <arg name="speed" value="1.5" />
    <arg name="reverse" value="false" />
</include>
```

### Example

```xml
<include file="ltu_actor_waypoint.launch">
    <arg name="speed" value="0.7" />
    <arg name="reverse" value="false" />
</include>
```
