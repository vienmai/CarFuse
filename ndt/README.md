# Build the docker image
```bash
docker build -t ndt:dev .
```

# Run the container

```bash
compose up -d
```

Next, issue the following command to get access to the running container:
```bash
docker exec -it ndt_locator /bin/bash
```
Remember to `source` the `ros_entrypoint.sh` at top level directory or source
the `entrypoint.sh` in the working directory:
```bash
source entrypoint.sh
```
This is because `docker exec` won't execute the `entrypoint` as `docker run`
does. Also, using `source` instead of `./entrypoint`, this will run the file in the
current shell.

# Build and test the package

```bash
colcon build --symlink-install
```

Or just build the `ndt` package:
```bash
colcon build --symlink-install --packages-select ndt_locator
```

Check if the package is correctly installed and built by running `colcon list` command.
It should show the package name with installed next to it.

```
./build/ndt_locator/test/ndt_test
colcon test --packages-select ndt_locator
```

# Run the package

```bash
source /ros2_ws/install/setup.bash
source /ros2_ws/install/local_setup.bash
```

Then you can launch the file using the following command:
```bash
ros2 launch ndt_locator ndt.launch.py
```

You can try to run the executable directly using the command
```bash
ros2 run ndt_locator ndt_node
```
to see if the executable runs correctly or not.


Run the `rosbag` to generate lidar scans:
```
ros2 bag play install/ndt_locator/share/ndt_locator/maps/court_yard_wed_repeat_night_2021-03-03-19-07-18 --remap /os_cloud_node/points:=/scan
```

# Rviz

```bash
ros2 run rviz2 rviz2
```

# Debugging

- Show the list of nodes and topics:
```bash
ros2 topic list
ros2 node list
```

- Show info of a particular node or topic:
```bash
ros2 topic info /topic_name
ros2 topic info /node_name
```

- Show rosbag info:
```bash
 ros2 bag info path_to_rosbag
```

# Mics

Convert rosbag to rosbag2
```
pip install rosbags
# Convert "foo.bag", result will be "foo/"
rosbags-convert foo.bag
```
check more [here](https://ternaris.gitlab.io/rosbags/index.html)
