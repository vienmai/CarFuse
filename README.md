# CarFuse: Multi-sensor Fusion for State Estimation

CarFuse is a ROS2 package that implements multi-sensor fusion for state estimation of a vehicle. It fuses the measurements obtained from various sensors such as LIDAR, IMU and GNSS to provide robust and accurate state estimates using an Error State Extended Kalman Filter (ESEKF). The estimated poses from the LIDAR measurements are obtained using Normal Distribution Transform (NDT) method.

## Project Structure

The project is organized into the following packages:

* `ndt`: implements the NDT-based lidar localization method. It includes a class for performing the localization and a demo script that demonstrates how to use the class to estimate the vehicle's pose.
* `imu`: implements the processing of IMU measurements. It includes a class for filtering and transforming IMU readings into a useful form for the ESEKF.
* `gnss`: implements the processing of GNSS measurements. It includes a class for filtering and transforming GNSS readings into a useful form for the ESEKF.
* `esekf`: implements the Error State Kalman Filter for fusing the NDT, IMU, and GNSS measurements. It includes a class for performing the fusion and a demo script that demonstrates how to use the filter.
* `launch`: contains launch files for starting the complete system.

# Prerequisites

To run this project, the only thing you need to install is `Docker`.

## Installation
The project includes a Dockerfile that can be used to build a Docker image containing all the necessary dependencies. To build the image, run the following command in the root of the project:

To run the project using Docker, follow these steps:

1. Clone this repository:
```bash
git clone https://github.com/vienmai/CarFuse.git
````

2. Build the Docker images:
```bash
docker build -t <image_name> .
```

3. Start the containers in detached mode:
```bash
docker-compose up -d
```

4. In a new terminal, run the launch file to start the nodes:
```bash
docker exec -it <container_name> /bin/bash
source entrypoint.bash
ros2 launch ndt_localization ndt_localization.launch.py
```
5. Start the other nodes in similar fashion, e.g.:
```
docker exec -it <container_name> /bin/bash/
ros2 launch imu imu.launch.py
```

This will start the NDT localization, IMU processing, GNSS processing, and ESEKF fusion nodes. You can use the rviz visualization tool to see the results of the estimation.


## Note

The reference frames used in the project are defined in the following way:

* `map`: The map reference frame for the NDT localization.
* `laser`: The frame of the lidar sensor, which is usually mounted on the vehicle's body.
* `imu`: The frame of the IMU sensor, which is usually also mounted on the vehicle's body.
* `gnss`: The WGS84 earth-centered earth-fixed (ECEF) reference frame.
* `base_link`: The body frame of the vehicle, which is used as the reference for all other sensors.

When fusing measurements from multiple sensors, it is important to be aware of the different reference frames and to properly transform measurements from one frame to another as needed. In this project, ESEKF is used to fuse the measurements obtained from different sensors, which are transformed to the `base_link` frame prior to the fusion process. The fused state estimates are then transformed back to the map frame for visualization and comparison with the ground truth.