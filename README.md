- Map frame: This is the frame of reference for the pre-built map that is used for scan matching.
- Lidar frame: This is the frame of reference for the lidar sensor, which is usually mounted on the vehicle.
- Base frame: This is the frame of reference for the vehicle's base, which is typically the center of the rear axle.
- Current pose frame: This is the frame of reference for the current estimated pose of the vehicle, which is calculated using the scan matching and the initial pose estimates.
- Initial pose frame: This is the frame of reference for the initial estimate of the vehicle's pose, which is typically obtained from a GPS sensor or another localization method.
- odom frame: This is the frame of reference for the odometry estimate of the vehicle's pose, which is usually calculated based on the vehicle's encoders.


# Summary

```The NDTLocalization class is designed to use scan matching with the Normal Distribution Transform (NDT) algorithm to estimate the vehicle's pose with respect to a pre-built map. The class subscribes to the following topics:

scan: for receiving lidar scans
initialpose: for receiving the initial estimated pose of the vehicle
The class has the following reference frames:

map: the reference frame for the pre-built map
base_link: the reference frame for the vehicle's body
initial_pose: the reference frame for the initial estimated pose of the vehicle
The class has the following member variables:

ndt_: an instance of the NDT scan matcher
map_ptr_: a unique pointer to the pre-built map in the form of a PointCloud2
tf_listener_: an instance of the tf2 listener
current_pose_: the current estimated pose of the vehicle, represented as a tf2::Transform and initially set to the initial_pose
map_to_base_: the transformation from the map frame to the base_link frame
The class has the following member functions:

scan_callback: callback function for the scan topic, performs scan matching with the NDT algorithm, calculates the current estimated pose of the vehicle and publishes it as a transform and as a PoseStamped message
initialpose_callback: callback function for the initialpose topic, updates the current_pose_ with the received initial pose
publish_transform: publishes the current estimated pose of the vehicle as a transform
publish_pose: publishes the current estimated pose of the vehicle as a PoseStamped message
The class also has the following callback functions that are related to the publisher and listener

run(): a method that runs a loop to handle the publisher and listener
It is also a good practice to use ros::NodeHandle in the class, It is used to create a ROS node that allows you to communicate with other nodes, advertise or subscribe to topics and services, and access the ROS parameters.

In the class, I used rclcpp::Publisher<sensor_msgs::msg::Type>::SharedPtr instead of ros::Publisher because it is ROS 2 specific publisher.

It's important to note that the current_pose_ variable is in the map frame and it is updated in each callback function, so it's not a huge source of error as long as the initial pose and the scan matching are accurate. The map_to_base_ variable is used to store the transformation from the map frame to the base_link frame, it's used to transform the current_pose_ from the map frame to the base_link frame so that it can be published and used by other nodes in the system.
```# CarFuse
