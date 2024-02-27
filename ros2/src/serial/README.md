# Serial Communication

The node `bidirectional` subscribes to
- `joy/cmd_vel` (geometry_msgs/Twist)

...sends the received data to the Arduino via serial communication and publishes the Arduino's response to
- TODO

## Run the node

```bash
ros2 run serial bidirectional --ros-args -p port:=/dev/ttyACM0
```

Replace `/dev/ttyACM0` with the port your Arduino is connected to.

