/*
 *  oCam.cpp
 *
 *  Created on: Oct 6, 2015
 *      Author: gnohead
 *
 *  Modified on: Sep 12, 2024
 *      Modified by: Yoon SoJeong
 *      Changes: Migrated from ROS1 to ROS2 Humble.
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <boost/thread.hpp>

#include "withrobot_camera.hpp"


class Camera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

private:
    int width_;
    int height_;
    int dev_num;
    bool flag0144 = false;
    std::vector<Withrobot::usb_device_info> dev_list;
    std::string devPath_;

public:

    Camera(int resolution, double frame_rate): camera(NULL) {

        enum_dev_list(dev_list);

        camera = new Withrobot::Camera(devPath_.c_str());

        for (int i=0; i < dev_num; i++) {
            if (flag0144 = true)
            {
                if (resolution == 0) { width_ = 1280; height_ = 800;}
                if (resolution == 1) { width_ = 1280; height_ = 720;}
                if (resolution == 2) { width_ = 640; height_  = 480;}
                if (resolution == 3) { width_ = 640; height_  = 400;}
                if (resolution == 4) { width_ = 320; height_  = 240;}
            }
            else{
                if (resolution == 0) { width_ = 1280; height_ = 960;}
                if (resolution == 1) { width_ = 1280; height_ = 720;}
                if (resolution == 2) { width_ = 640; height_  = 480;}
                if (resolution == 3) { width_ = 320; height_  = 240;}
            }

        }
        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','R','B','G'), 1, (unsigned int)frame_rate);

        /*
         * get current camera format (image size and frame rate)
         */
        camera->get_current_format(camFormat);
        camFormat.print();

        /* Withrobot camera start */
        camera->start();
	}

    ~Camera() {
        camera->stop();
        delete camera;

	}

    void enum_dev_list(std::vector<Withrobot::usb_device_info> dev_list)
    {
        /* enumerate device(UVC compatible devices) list */
        // std::vector<Withrobot::usb_device_info> dev_list;
        dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) {

            if (dev_list[i].product == "oCam-1CGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1CGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1CGN-U-T2")
            {
                devPath_ = dev_list[i].dev_node;
                flag0144 = true;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U-T2")
            {
                devPath_ = dev_list[i].dev_node;
                flag0144 = true;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCam-1MGN-U-T")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
        }
    }

    void uvc_control(int exposure, int gain, int blue, int red, bool ae)
    {
        /* Exposure Setting */
        camera->set_control("Exposure (Absolute)", exposure);

        /* Gain Setting */
        camera->set_control("Gain", gain);

        /* White Balance Setting */

        camera->set_control("Blue Balance", blue);
        camera->set_control("Red Balance", red);

        /* Auto Exposure Setting */
        if (ae)
            camera->set_control("Exposure, Auto", 0x3);
        else
            camera->set_control("Exposure, Auto", 0x1);

    }

    bool getImages(cv::Mat &image) {

        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        cv::Mat dstImg;

        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1)
        {
            cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
            image = dstImg;

            return true;
        } else {
            return false;
        }
	}
};

/**
 * @brief       the camera ros warpper class
 */
class oCamROS : public rclcpp::Node{

private:
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_;

    //ros::NodeHandle nh;
    std::string camera_frame_id_;
    Camera* ocam;
    /**
     * @brief      { publish camera info }
     *
     * @param[in]  pub_cam_info  The pub camera information
     * @param[in]  cam_info_msg  The camera information message
     * @param[in]  now           The now
     */
    void publishCamInfo(const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr& pub_cam_info,
                        sensor_msgs::msg::CameraInfo& cam_info_msg, rclcpp::Time now) {
        cam_info_msg.header.stamp = now;
        pub_cam_info->publish(cam_info_msg);
    }

    /**
     * @brief      { publish image }
     *
     * @param[in]  img           The image
     * @param      img_pub       The image pub
     * @param[in]  img_frame_id  The image frame identifier
     * @param[in]  t             { parameter_description }
     */
    void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, rclcpp::Time t) {
        cv_bridge::CvImage cv_image;
        cv_image.image = img;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        img_pub.publish(cv_image.toImageMsg());
    }

    void device_poll() {
    // 설정을 위한 파라미터 콜백 대신 ROS2의 파라미터 콜백을 사용


    auto param_callback_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult {
            return this->on_parameter_change(parameters);  // 콜백 처리 함수
        }
    );


    // image_transport 초기화
    image_transport::ImageTransport it(shared_from_this());
    auto camera_image_pub = it.advertise("camera/image_raw", 1);


    auto camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);


    sensor_msgs::msg::CameraInfo camera_info;

    RCLCPP_INFO(this->get_logger(), "Loading from ROS calibration files");

    // camera_info_manager 초기화
    camera_info_manager::CameraInfoManager info_manager(shared_from_this().get());

    info_manager.loadCameraInfo("package://ocam/config/camera.yaml");
    camera_info = info_manager.getCameraInfo();

    camera_info.header.frame_id = camera_frame_id_;

    RCLCPP_INFO(this->get_logger(), "Got camera calibration files");

    // 이미지 처리 루프
    cv::Mat camera_image;
    rclcpp::Rate r(frame_rate_);

    while (rclcpp::ok())
    {
        rclcpp::Time now = this->now();

        if (!ocam->getImages(camera_image)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ROS2에서 usleep 대신 사용
            continue;
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Success, found camera");
        }

        // 퍼블리셔에 이미지가 구독되고 있는지 확인
        if (camera_image_pub.getNumSubscribers() > 0) {
            publishImage(camera_image, camera_image_pub, "camera_frame", now);
        }

        if (camera_info_pub->get_subscription_count() > 0) {
            publishCamInfo(camera_info_pub, camera_info, now);
        }

            if (show_image_) {
            //cv::imshow("image", camera_image);
            //cv::waitKey(10);
        }


        r.sleep();

    }
     RCLCPP_DEBUG(this->get_logger(), "This is a debug message.");

}


    rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &param : parameters) {
        if (param.get_name() == "exposure") {
            ocam->uvc_control(param.as_int(), gain_, wb_blue_, wb_red_, autoexposure_);
        } else if (param.get_name() == "gain") {
            gain_ = param.as_int();
            ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        } else if (param.get_name() == "wb_blue") {
            wb_blue_ = param.as_int();
            ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        } else if (param.get_name() == "wb_red") {
            wb_red_ = param.as_int();
            ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        } else if (param.get_name() == "auto_exposure") {
            autoexposure_ = param.as_bool();
            ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        }

    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}


    // void callback(ocam::camConfig &config, uint32_t level) {
    //     ocam->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
    // }


public:
    /**
	 * @brief      { function_description }
	 *
	 * @param[in]  resolution  The resolution
	 * @param[in]  frame_rate  The frame rate
     */
    oCamROS() : Node("ocam_node"){
        //ros::NodeHandle priv_nh("~");

        /* default parameters */
        this->declare_parameter<int>("resolution", 0);
        this->declare_parameter<double>("frame_rate", 60.0);
        this->declare_parameter<int>("exposure", 100);
        this->declare_parameter<int>("gain", 150);
        this->declare_parameter<int>("wb_blue", 200);
        this->declare_parameter<int>("wb_red", 160);
        this->declare_parameter<std::string>("camera_frame_id", "camera");
        this->declare_parameter<bool>("show_image", true);
        this->declare_parameter<bool>("auto_exposure", false);


        /* get parameters */
        this->get_parameter("resolution", resolution_);
        this->get_parameter("frame_rate", frame_rate_);
        this->get_parameter("exposure", exposure_);
        this->get_parameter("gain", gain_);
        this->get_parameter("wb_blue", wb_blue_);
        this->get_parameter("wb_red", wb_red_);
        this->get_parameter("camera_frame_id", camera_frame_id_);
        this->get_parameter("show_image", show_image_);
        this->get_parameter("auto_exposure", autoexposure_);

        /* initialize the camera */
        ocam = new Camera(resolution_, frame_rate_);
        ocam->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);

        RCLCPP_INFO(this->get_logger(), "Initialized the camera");

	}

    void start_device_poll()
    {
        auto self = std::dynamic_pointer_cast<oCamROS>(shared_from_this());
        std::thread poll_thread(&oCamROS::device_poll, self);
        poll_thread.detach(); // 백그라운드에서 실행
    }

    ~oCamROS() {
        delete ocam;
    }
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<oCamROS>();
    node->start_device_poll();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

