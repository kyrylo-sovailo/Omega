#include <omega/camera.h>
#include <omega/omega.h>
#include <cv_bridge/cv_bridge.h>

void omega::Camera::_update(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    const Eigen::Matrix<double, 3, 4> p = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>::Map(msg->P.data());
    const Eigen::Matrix<double, 3, 3> k = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Map(msg->K.data());
    Eigen::Matrix4d new_matrix;
    new_matrix.block<3, 4>(0, 0) = k * p;
    new_matrix.block<1, 4>(3, 0) = Eigen::Vector4d(0, 0, 0, 1);
    if (matrix != new_matrix)
    {
        matrix = new_matrix;
        matrix_inverse = new_matrix.inverse();
    }
}

omega::Camera::Camera(ros::NodeHandle *node, Omega *owner) : _owner(owner)
{
    //Config
    OMEGA_CONFIG_VECTOR("camera/base_position", 3, base_position);
    OMEGA_CONFIG("camera/blur_strength", blur_strength);
    OMEGA_CONFIG("camera/blur_size", blur_size);
    boost::shared_ptr<const sensor_msgs::CameraInfo> msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("sensor/camera/camera_info", ros::Duration(5.0));
    if (msg == nullptr) { const char *error = "Failed to receive camera info"; ROS_ERROR(error); throw std::runtime_error(error); }
    _update(msg);

    //Technical
    image_transport::ImageTransport image_transfer(*node);
    _sub = image_transfer.subscribe("sensor/camera/image_rect_color", 1, &Omega::image_update, owner);
    _sub_info = node->subscribe("sensor/camera/camera_info", 1, &Camera::_update, this);
}

void omega::Camera::update(const sensor_msgs::ImageConstPtr &msg, cv::Mat &image)
{
    cv_bridge::CvImagePtr image_pointer = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::GaussianBlur(image_pointer->image, image_pointer->image, cv::Size(blur_size, blur_size), blur_strength);
    image = image_pointer->image;
}