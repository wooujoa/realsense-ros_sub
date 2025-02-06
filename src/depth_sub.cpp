#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DepthImageSubscriber : public rclcpp::Node
{
public:
  DepthImageSubscriber()
  : Node("depth_image_subscriber"), it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){})) // 안전한 ImageTransport 생성
  {
    image_sub_ = it_.subscribe("/camera/camera/depth/image_rect_raw", 10,
      std::bind(&DepthImageSubscriber::imageCallback, this, std::placeholders::_1));
  }

private:
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // Step 1: 유효한 크기인지 확인
  if (msg->height == 0 || msg->width == 0 || msg->step == 0) {
    RCLCPP_ERROR(this->get_logger(), "Received an image with invalid dimensions: height=%d, width=%d, step=%d",
      msg->height, msg->width, msg->step);
    return;
  }

  // Step 2: 데이터가 비어있는지 확인
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received an empty image message.");
    return;
  }

  try {
    // Step 3: cv_bridge를 이용해 Depth 이미지 변환
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");  // Depth 카메라 데이터는 16-bit 단일 채널

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Converted depth image is empty.");
      return;
    }

    // Step 4: Depth 값을 8-bit로 변환 (시각화를 위해 정규화)
    cv::Mat depth_display;
    double min_val, max_val;
    cv::minMaxIdx(cv_ptr->image, &min_val, &max_val);  // 최소/최대 Depth 값 찾기
    cv_ptr->image.convertTo(depth_display, CV_8UC1, 255.0 / max_val);  // 0~255 정규화

    // Step 5: 컬러맵 적용 (Depth 데이터를 보기 쉽게 변환)
    cv::Mat depth_colormap;
    cv::applyColorMap(depth_display, depth_colormap, cv::COLORMAP_JET);  // JET 컬러맵 적용

    // Step 6: OpenCV로 출력
    cv::imshow("Depth Camera Image", depth_colormap);
    cv::waitKey(1);

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthImageSubscriber>(); // 반드시 shared_ptr로 생성
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
