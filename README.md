# CarFuse: Multi-sensor Fusion for State Estimation

CarFuse is a ROS2 package that implements multi-sensor fusion for state estimation of a vehicle. It fuses the measurements obtained from various sensors such as LIDAR, IMU and GNSS to provide robust and accurate state estimates using an Error State Extended Kalman Filter (ESEKF). The estimated poses from the LIDAR measurements are obtained using Normal Distribution Transform (NDT) method.

## Project Structure

The project is organized into the following packages:

* `ndt`: implements the NDT-based lidar localization method. It includes a class for performing the localization and a demo script that demonstrates how to use the class to estimate the vehicle's pose.
* `imu`: implements the processing of IMU measurements. It includes a class for filtering and transforming IMU readings into a useful form for the ESKF.
* `gnss`: implements the processing of GNSS measurements. It includes a class for filtering and transforming GNSS readings into a useful form for the ESKF.
* `eskf`: implements the Error State Kalman Filter for fusing the NDT, IMU, and GNSS measurements. It includes a class for performing the fusion and a demo script that demonstrates how to use the filter.
* `launch`: contains launch files for starting the complete system.

# Prerequisites
```
- Docker
- Docker Compose
```

## Installation
The project includes a Dockerfile that can be used to build a Docker image containing all the necessary dependencies. To build the image, run the following command in the root of the project:

To run the project using Docker, follow these steps:

1. Clone this repository:
```bash
git clone https://github.com/<your-username>/multi-sensor-fusion-state-estimation.git
````

2. Build the Docker images:
```bash
docker build -t <image_name> .
```

3. Start the containers:
```bash
docker-compose up
```

4. In a new terminal, run the launch file to start the nodes:
```bash
docker exec -it multi-sensor-fusion-state-estimation_ndt-localization_1 bash
source /opt/ros/dashing/setup.bash
ros2 launch ndt_localization ndt_localization.launch.py
```
5. Start the other nodes in similar fashion, e.g.:
```
docker exec -it multi-sensor-fusion-state-estimation_imu_1 bash
source /opt/ros/dashing/setup.bash
ros2 launch imu imu.launch.py
```

This will start the NDT localization, IMU processing, GNSS processing, and ESKF fusion nodes. You can use the rviz visualization tool to see the results of the estimation.