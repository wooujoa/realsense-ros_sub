#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber"), it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){})) // 안전한 ImageTransport 생성
  {
    image_sub_ = it_.subscribe("/camera/camera/color/image_rect_raw", 10,
      std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
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
    // Step 3: cv_bridge로 변환
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    // Step 4: OpenCV 이미지가 비정상적으로 큰 경우를 방지
    int max_width = 1920;   // 최대 허용 가로 크기
    int max_height = 1080;  // 최대 허용 세로 크기
    cv::Mat resized_image;

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Converted image is empty.");
      return;
    }

    // Step 5: 이미지 크기 제한
    if (cv_ptr->image.cols > max_width || cv_ptr->image.rows > max_height) {
      RCLCPP_WARN(this->get_logger(), "Resizing image from [%dx%d] to [%dx%d]",
                  cv_ptr->image.cols, cv_ptr->image.rows, max_width, max_height);

      // 비율 유지하면서 크기 조절
      double scale_x = static_cast<double>(max_width) / cv_ptr->image.cols;
      double scale_y = static_cast<double>(max_height) / cv_ptr->image.rows;
      double scale = std::min(scale_x, scale_y);

      int new_width = static_cast<int>(cv_ptr->image.cols * scale);
      int new_height = static_cast<int>(cv_ptr->image.rows * scale);

      cv::resize(cv_ptr->image, resized_image, cv::Size(new_width, new_height));
    } else {
      resized_image = cv_ptr->image;
    }

    // Step 6: 정상적인 경우 OpenCV로 출력
    cv::imshow("USB Camera Image", resized_image);
    cv::waitKey(1);

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}


  image_transport::ImageTransport it_; // 멤버 변수로 선언
  image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>(); // 반드시 shared_ptr로 생성
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}