# tf2_lookup_server

Additional requirements:
- [Eigen3](https://eigen.tuxfamily.org), e.g. via ```sudo apt install libeigen3-dev```

```
colcon build --packages-select tf2_lookup_server && source install/setup.bash
```

# Nodes
## tf2_lookup_server_node
Requests can be made for looking up instantaneous transforms, or a transform that is calculated as the average over a window.

Uses tf2_msgs/action/LookupTransform:
- target_frame, source_frame, source_time: as usual
- timeout: either a timeout for one-shot lookups, or the duration of a lookup window (can be negative for averaging into the past)
- target_time, fixed_frame: unused
- advanced: if true, averages over duration specified by timeout

```
ros2 run tf2_lookup_server tf2_lookup_server_node
```
Additional parameters:

- ```buffer_duration``` (default: 10.0): duration (s) of TF buffer
- ```hz``` (default: 50): lookup loop frequency

# Examples
## Lookup server test client
```
cd test
python3 lookup_client_test.py request.json
```
An example of querying the lookup_server. request.json is configured for a lookup from ```pelvis``` to ```head```, averaged over the next few seconds until the timeout duration has elapsed.