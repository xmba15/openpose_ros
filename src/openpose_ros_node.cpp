#define USE_CAFFE

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

#include <chrono>
#include <string>
#include <opencv2/core/core.hpp>

// 3rdparty dependencies
#include <gflags/gflags.h> // DEFINE_bool, DEFINE_int32, DEFINE_int64, DEFINE_uint64, DEFINE_double, DEFINE_string
#include <glog/logging.h> // google::InitGoogleLogging
// OpenPose dependencies
#include <openpose/headers.hpp>

#include <openpose_ros_msgs/BodypartDetection.h>
#include <openpose_ros_msgs/PeoplePose.h>
#include <openpose_ros_msgs/PeoplePoseArray.h>

DEFINE_int32(logging_level,             4,              "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while"
                                                        " 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for"
                                                        " low priority messages and 4 for important ones.");
DEFINE_double(scale_gap,                0.3,            "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1."
                                                        " If you want to change the initial scale, you actually want to multiply the"
                                                        " `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number,              1,              "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending,           false,          "If blending is enabled, it will merge the results with the original frame. If disabled, it"
                                                        " will only display the results.");
DEFINE_double(render_threshold,         0.05,           "Only estimated keypoints whose score confidences are higher than this threshold will be"
                                                        " rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;"
                                                        " while small thresholds (~0.1) will also output guessed and occluded keypoints, but also"
                                                        " more false positives (i.e. wrong detections).");
DEFINE_double(alpha_pose,               0.6,            "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will"
                                                        " hide it. Only valid for GPU rendering.");

std::map<unsigned int, std::string> g_bodypart_map;

template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& param_name, T default_value)
{
  T value;
  if (nh.hasParam(param_name))
    {
      nh.getParam(param_name, value);
    }
  else
    {
      ROS_WARN_STREAM("Parameter '" << param_name << "' not found, defaults to '" << default_value << "'");
      value = default_value;
    }
  return value;
}

openpose_ros_msgs::BodypartDetection getBodyPartDetectionFromArrayAndIndex(const op::Array<float>& array, size_t idx)
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.x = array[idx];
  bodypart.y = array[idx+1];
  bodypart.confidence = array[idx+2];
  return bodypart;
}

openpose_ros_msgs::BodypartDetection getNANBodypart()
{
  openpose_ros_msgs::BodypartDetection bodypart;
  bodypart.confidence = NAN;
  return bodypart;
}

std::map<unsigned int, std::string> getBodyPartMapFromPoseModel(const op::PoseModel& pose_model)
{
  if (pose_model == op::PoseModel::COCO_18)
  {
    return op::POSE_COCO_BODY_PARTS;
  }
  else if (pose_model == op::PoseModel::MPI_15 || pose_model == op::PoseModel::MPI_15_4)
  {
    return op::POSE_MPI_BODY_PARTS;
  }
  else
  {
    ROS_FATAL("Invalid pose model, not map present");
    exit(1);
  }
}

class SkeletonDetection
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_img_ptr_;
  ros::Publisher skeleton_pub;
  
public:
  std::string image_topic;
  std::string resolution;
  std::string net_resolution;
  std::string model_pose;
  std::string model_folder;
  int num_gpu_start;
  std_msgs::Header img_msg_header;
  
  SkeletonDetection(): it_(nh_)
  {
    image_topic = getParam(nh_, "camera_topic", std::string("/camera/rgb/image_rect_color"));
    image_sub_ = it_.subscribe(image_topic, 1, &SkeletonDetection::convertImage, this);
    cv_img_ptr_ = nullptr;
    image_pub_ = it_.advertise("/openpose_ros/debug_img", 1);
    skeleton_pub = nh_.advertise<openpose_ros_msgs::PeoplePoseArray>("/openpose_ros/skeleton", 1000);

    resolution = getParam(nh_, "resolution", std::string("640x480"));
    net_resolution = getParam(nh_, "net_resolution", std::string("656x368"));
    model_pose = getParam(nh_, "model_pose", std::string("COCO"));
    std::string package_path = ros::package::getPath("openpose_ros");
    std::string folder_location = package_path + "/openpose/models/";
    model_folder = getParam(nh_, "model_folder", folder_location);
    num_gpu_start = getParam(nh_, "num_gpu_start", 0);
  }

  ~SkeletonDetection(){}

  void convertImage(const sensor_msgs::ImageConstPtr& msg)
  {
    try
      {
	cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	img_msg_header = msg->header;
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
  }

  cv_bridge::CvImagePtr& getCvImagePtr()
  {
    return cv_img_ptr_;
  }

  void publishSkeleton(openpose_ros_msgs::PeoplePoseArray people_pose_array_msg)
  {
    skeleton_pub.publish(people_pose_array_msg);
  }

  void publishImage(cv::Mat img)
  {
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    image_pub_.publish(img_msg);
  }

};

int openPoseROSTutorial()
{
    // Step 0 - Initialize the image subscriber
    SkeletonDetection ris;

    op::log("OpenPose ROS Node", op::Priority::High);
    // ------------------------- INITIALIZATION -------------------------
    // Step 1 - Set logging level
        // - 0 will output all the logging messages
        // - 255 will output nothing
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 2 - Read Google flags (user defined configuration)
    // outputSize
    const auto outputSize = op::flagsToPoint(ris.resolution, "640x480");
    // netInputSize
    // const auto netInputSize = op::flagsToPoint(ris.net_resolution, "656x368");
    const auto netInputSize = op::flagsToPoint(ris.net_resolution, "640x480");
    // netOutputSize
    // const auto netOutputSize = netInputSize;
    const auto netOutputSize = outputSize;
    // poseModel
    const auto poseModel = op::flagsToPoseModel(ris.model_pose);
    g_bodypart_map = getBodyPartMapFromPoseModel(poseModel);

    // Check no contradictory flags enabled
    if (FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (FLAGS_scale_gap <= 0. && FLAGS_scale_number > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);
    // Logging
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    // Step 3 - Initialize all required classes
    op::CvMatToOpInput cvMatToOpInput{netInputSize, FLAGS_scale_number, (float)FLAGS_scale_gap};
    op::CvMatToOpOutput cvMatToOpOutput{outputSize};
    op::PoseExtractorCaffe poseExtractorCaffe{netInputSize, netOutputSize, outputSize, FLAGS_scale_number, poseModel,
                                              ris.model_folder, ris.num_gpu_start};
    op::PoseRenderer poseRenderer{netOutputSize, outputSize, poseModel, nullptr, (float)FLAGS_render_threshold,
                                  !FLAGS_disable_blending, (float)FLAGS_alpha_pose};
    op::OpOutputToCvMat opOutputToCvMat{outputSize};
    // Step 4 - Initialize resources on desired thread (in this case single thread, i.e. we init resources here)
    poseExtractorCaffe.initializationOnThread();
    poseRenderer.initializationOnThread();


    int frame_count = 0;
    const std::chrono::high_resolution_clock::time_point timerBegin = std::chrono::high_resolution_clock::now();

    ros::spinOnce();

    // Step 6 - Continuously process images from image subscriber
    while (ros::ok())
    {
        // ------------------------- POSE ESTIMATION AND RENDERING -------------------------
        // Step 1 - Get cv_image ptr and check that it is not null
        cv_bridge::CvImagePtr cvImagePtr = ris.getCvImagePtr();
        if(cvImagePtr != nullptr)
        {
            cv::Mat inputImage = cvImagePtr->image;
    
            // Step 2 - Format input image to OpenPose input and output formats
            op::Array<float> netInputArray;
            std::vector<float> scaleRatios;
            std::tie(netInputArray, scaleRatios) = cvMatToOpInput.format(inputImage);
            double scaleInputToOutput;
            op::Array<float> outputArray;
            std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput.format(inputImage);
            // Step 3 - Estimate poseKeypoints
            poseExtractorCaffe.forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
            const auto poseKeypoints = poseExtractorCaffe.getPoseKeypoints();
            // Step 4 - Render poseKeypoints
            poseRenderer.renderPose(outputArray, poseKeypoints);
            // Step 5 - OpenPose output format to cv::Mat
            auto outputImage = opOutputToCvMat.formatToCvMat(outputArray);

	    // publish skeleton
	    int num_people = poseKeypoints.getSize(0);
	    int num_bodyparts = poseKeypoints.getSize(1);
	    ROS_INFO("num people: %d", num_people);

	    openpose_ros_msgs::PeoplePoseArray people_pose_array_msg;
	    people_pose_array_msg.header = ris.img_msg_header;

	    for (size_t person_idx = 0; person_idx < num_people; person_idx++)
	      {
		ROS_INFO("Person ID: %zu", person_idx);

		openpose_ros_msgs::PeoplePose people_pose_msg;

		for (size_t bodypart_idx = 0; bodypart_idx < num_bodyparts; bodypart_idx++)
		  {
		    size_t final_idx = 3*(person_idx*num_bodyparts + bodypart_idx);

		    std::string body_part_string = g_bodypart_map[bodypart_idx];
		    openpose_ros_msgs::BodypartDetection bodypart_detection = getBodyPartDetectionFromArrayAndIndex(poseKeypoints, final_idx);
		    
		    people_pose_msg.limb_names.push_back(body_part_string);

		    bodypart_detection.confidence >= 0 ? people_pose_msg.confidences.push_back(bodypart_detection.confidence) : people_pose_msg.confidences.push_back(NAN);
		    geometry_msgs::Pose bodypart_pose;
		    bodypart_pose.position.x = bodypart_detection.x;
		    bodypart_pose.position.y = bodypart_detection.y;
		    bodypart_pose.position.z = 0;
		    people_pose_msg.poses.push_back(bodypart_pose);

		    ROS_INFO("body part: %s", body_part_string.c_str());
		    ROS_INFO("(x, y, confidence): %i, %i, %f", bodypart_detection.x, bodypart_detection.y, bodypart_detection.confidence);

		  }
		
		people_pose_array_msg.poses.push_back(people_pose_msg);

	      }

	    ris.publishSkeleton(people_pose_array_msg);

            // ------------------------- SHOWING RESULT AND CLOSING -------------------------
            // Step 1 - Show results

	    ris.publishImage(outputImage);
            frame_count++;
        }
        ros::spinOnce();
    }

    // Measuring total time
    const double totalTimeSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>
                              (std::chrono::high_resolution_clock::now()-timerBegin).count() * 1e-9;
    const std::string message = "Real-time pose estimation demo successfully finished. Total time: " 
                                + std::to_string(totalTimeSec) + " seconds. " + std::to_string(frame_count)
                                + " frames processed. Average fps is " + std::to_string(frame_count/totalTimeSec) + ".";
    op::log(message, op::Priority::Max);

    // Return successful message
    return 0;
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging("openpose_ros_node");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "openpose_ros_node");

  return openPoseROSTutorial();
}
