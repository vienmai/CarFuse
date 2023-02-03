#include "ndt/ndt.hpp"

namespace localization {
NDTLocalization::NDTLocalization() : Node("ndt"), tf_buffer_() {
	// NDT parameters
	double resolution = this->get_parameter_or("resolution", 1.0);
	double stepsize = this->get_parameter_or("stepsize", 0.1);
	double epsilon = this->get_parameter_or("epsilon", 0.1);
	size_t maxiters = this ->get_parameter_or("maxiters", 20);

	// Reference Frames
	map_frame_ = this->get_parameter_or("map_frame", std::string("map"));
	vehicle_frame_ = this->get_parameter_or("vehicle_frame", std::string("base_link"));
	laser_frame_ = this->get_parameter_or("laser_frame", std::string("laser"));
	initial_pose_frame_ = this->get_parameter_or("initial_pose_frame", std::string("GNSS"));

	// Topics
	std::string scan_topic = this->get_parameter_or("scan_topic", std::string("/scan"));
	std::string initial_pose_topic = this->get_parameter_or("initial_pose_topic", std::string("/initial_pose"));
	std::string map_topic = this->get_parameter_or("map_topic", std::string("/map"));
	std::string pose_topic = this->get_parameter_or("pose_topic", std::string("/pose"));
	std::string cloud_topic = this->get_parameter_or("cloud_topic", std::string("/cloud"));
	std::string tf_topic = this->get_parameter_or("tf_topic", std::string("/tf"));

	// Initialize the NDT scan matcher
	ndt_.setTransformationEpsilon(epsilon);
	ndt_.setStepSize(stepsize);
	ndt_.setResolution(resolution);
	ndt_.setMaximumIterations(maxiters);

	// Initialize the tf2 buffer and listener
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

	// Initialize the publisher for the current pose
	pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		pose_topic, rclcpp::QoS(rclcpp::KeepLast(10))
	);

	// Initialize the publisher for the transformed point cloud
	cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
		cloud_topic, rclcpp::QoS(rclcpp::KeepLast(10))
	);

	// Initialize the subscriber for the initial pose
	initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		initial_pose_topic,
		rclcpp::QoS(rclcpp::KeepLast(1)),
		std::bind(&NDTLocalization::initial_pose_callback, this, std::placeholders::_1)
	);

	// Initialize the subscriber for the lidar scan
	scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		scan_topic,
		rclcpp::QoS(rclcpp::KeepLast(10)),
		std::bind(&NDTLocalization::scan_callback, this, std::placeholders::_1)
	);

	// Initialize the subscriber for the map
	map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		map_topic,
		rclcpp::QoS(rclcpp::KeepLast(1)),
		std::bind(&NDTLocalization::map_callback, this, std::placeholders::_1)
	);
};

void NDTLocalization::scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pcd_msg) {
	RCLCPP_INFO(get_logger(), "Start scan callback.");

	// Convert the cloud message to a point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*pcd_msg, cloud);

	// Transform the cloud from sensor to vehicle frame
	transform_pointcloud(cloud, cloud, vehicle_frame_);
	RCLCPP_INFO(get_logger(), "PCD transformed to vehicle frame.");

	// Perform scan matching with NDT
	pcl::PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>(cloud));
	pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);
	ndt_.setInputSource(cloud_ptr);
	ndt_.setInputTarget(map_ptr_);
	ndt_.align(*output_cloud, current_pose_);

	RCLCPP_INFO(get_logger(), "Finished NDT scan matching.");

	// Update the current pose
	current_pose_ = ndt_.getFinalTransformation();
	Eigen::Matrix3f rmat = current_pose_.block<3, 3>(0, 0);
	Eigen::Vector3f tvec = current_pose_.block<3, 1>(0, 3);
	Eigen::Quaternionf quat(rmat);

  // Create a pose message to publish
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.stamp = pcd_msg->header.stamp;
	pose_msg.header.frame_id = map_frame_;
	pose_msg.pose.position.x = tvec(0);
	pose_msg.pose.position.y = tvec(1);
	pose_msg.pose.position.z = tvec(2);
	pose_msg.pose.orientation.x = quat.x();
	pose_msg.pose.orientation.y = quat.y();
	pose_msg.pose.orientation.z = quat.z();
	pose_msg.pose.orientation.w = quat.w();

	// Publish the results
	pose_pub_->publish(pose_msg);
	publish_transform(pose_msg);
	pose_vec_.push_back(pose_msg);

	RCLCPP_INFO(get_logger(), "End scan callback.");
}

void NDTLocalization::map_callback(
	const sensor_msgs::msg::PointCloud2::SharedPtr map_msg
) {
	RCLCPP_INFO(get_logger(), "Start map callback.");
	pcl::PCLPointCloud2 pcl_map;
	pcl_conversions::toPCL(*map_msg, pcl_map);
	map_ptr_ = std::make_shared<pcl::PointCloud<PointT>>();
	pcl::fromPCLPointCloud2(pcl_map, *map_ptr_);
	RCLCPP_INFO(get_logger(), "End map callback.");
}

void NDTLocalization::initial_pose_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr initial_pose_msg
) {
	RCLCPP_INFO(get_logger(), "Start intitial pose callback.");
	auto initpose_frame = initial_pose_msg->header.frame_id;
	RCLCPP_INFO(get_logger(), "Initial pose frame id: %s", initpose_frame.c_str());

  if (initpose_frame != map_frame_) {
    RCLCPP_INFO(get_logger(), "Transform initpose to %s frame", map_frame_.c_str());
		geometry_msgs::msg::TransformStamped tf_stamped;
		try {
			tf_stamped = tf_buffer_.lookupTransform(
				map_frame_,
				initpose_frame,
				tf2::TimePointZero
			);
		} catch (tf2::TransformException &ex) {
			RCLCPP_WARN(get_logger(), "%s", ex.what());
			return;
		}
		tf2::doTransform(*initial_pose_msg, *initial_pose_msg, tf_stamped);
	}

	// Publish the initial pose in map frame
	pose_pub_->publish(*initial_pose_msg);

	RCLCPP_INFO(get_logger(), "End initial pose callback.");
}

void NDTLocalization::transform_pointcloud(
	const pcl::PointCloud<PointT>& cloud_in,
	pcl::PointCloud<PointT>& cloud_out,
	const std::string& target_frame
) const {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
			tf_stamped = tf_buffer_.lookupTransform(
				target_frame, cloud_in.header.frame_id, tf2::TimePointZero
			);
    } catch (tf2::TransformException &ex) {
			RCLCPP_WARN(get_logger(), "%s", ex.what());
			return;
		}
    auto eigen_matrix = tf_stamped_to_eigen(tf_stamped);
    pcl::transformPointCloud(cloud_in, cloud_out, eigen_matrix);
}

void NDTLocalization::publish_transform(geometry_msgs::msg::PoseStamped& pose_msg) const {
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = pose_msg.header.stamp;
  tf_msg.header.frame_id = pose_msg.header.frame_id;
  tf_msg.child_frame_id = vehicle_frame_;

  tf_msg.transform.translation.x = pose_msg.pose.position.x;
  tf_msg.transform.translation.y = pose_msg.pose.position.y;
  tf_msg.transform.translation.z = pose_msg.pose.position.z;
  tf_msg.transform.rotation.x = pose_msg.pose.orientation.x;
  tf_msg.transform.rotation.y = pose_msg.pose.orientation.y;
  tf_msg.transform.rotation.z = pose_msg.pose.orientation.z;
  tf_msg.transform.rotation.w = pose_msg.pose.orientation.w;

  tf_broadcaster_->sendTransform(tf_msg);
}

// void publish_cloud(
//   const pcl::PointCloud<PointT>::Ptr& cloud,
//   const rclcpp::Time& stamp
// ) const {
//     sensor_msgs::msg::PointCloud2 cloud_msg;
//     pcl::toROSMsg(*cloud, cloud_msg);
//     cloud_msg.header.stamp = stamp;
//     cloud_pub_->publish(cloud_msg);
// }

std::vector<geometry_msgs::msg::PoseStamped> NDTLocalization::get_estimated_poses() const {
    return pose_vec_;
};

Eigen::Matrix4f NDTLocalization::tf_stamped_to_eigen(
  const geometry_msgs::msg::TransformStamped& tf_stamped
) const {
    Eigen::Matrix4f eigen_matrix;

    eigen_matrix(0, 0) = tf_stamped.transform.rotation.w;
    eigen_matrix(0, 1) = tf_stamped.transform.rotation.x;
    eigen_matrix(0, 2) = tf_stamped.transform.rotation.y;
    eigen_matrix(0, 3) = tf_stamped.transform.rotation.z;

    eigen_matrix(1, 0) = tf_stamped.transform.rotation.x;
    eigen_matrix(1, 1) = tf_stamped.transform.rotation.w;
    eigen_matrix(1, 2) = -tf_stamped.transform.rotation.z;
    eigen_matrix(1, 3) = tf_stamped.transform.rotation.y;

    eigen_matrix(2, 0) = tf_stamped.transform.rotation.y;
    eigen_matrix(2, 1) = tf_stamped.transform.rotation.z;
    eigen_matrix(2, 2) = tf_stamped.transform.rotation.w;
    eigen_matrix(2, 3) = -tf_stamped.transform.rotation.x;

    eigen_matrix(3, 0) = -tf_stamped.transform.rotation.z;
    eigen_matrix(3, 1) = tf_stamped.transform.rotation.y;
    eigen_matrix(3, 2) = -tf_stamped.transform.rotation.x;
    eigen_matrix(3, 3) = tf_stamped.transform.rotation.w;

    eigen_matrix(0, 3) = tf_stamped.transform.translation.x;
    eigen_matrix(1, 3) = tf_stamped.transform.translation.y;
    eigen_matrix(2, 3) = tf_stamped.transform.translation.z;

    return eigen_matrix;
}

}; // namespace localization