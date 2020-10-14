#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "uvc_camera/stereocamera.h"

using namespace sensor_msgs;


struct control_mod {
  uint32_t id;
  int32_t val;
  std::string name;

  control_mod(uint32_t id, int32_t val, const std::string& name) {
    this->id = id;
    this->val = val;
    this->name = name;
  }
};
typedef struct control_mod control_mod_t;


/* Rotate an 8-bit, 3-channel image by 180 degrees. */
static inline void rotate(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
  struct pixel_t {
    unsigned char r, g, b;
  };

  struct pixel_t *src = (pixel_t *) src_chr;
  struct pixel_t *dst = &(((pixel_t *) dst_chr)[pixels - 1]);

  for (int i = pixels; i != 0; --i) {
    *dst = *src;
    src++;
    dst--;
  }
}

namespace uvc_camera {

StereoCamera::StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh) :
  node(comm_nh), pnode(param_nh), it(comm_nh),
  left_info_mgr(ros::NodeHandle(comm_nh, "left"), "left_camera"),
  center_info_mgr(ros::NodeHandle(comm_nh, "center"), "center_camera"),
  center2_info_mgr(ros::NodeHandle(comm_nh, "center2"), "center2_camera"),
  five_info_mgr(ros::NodeHandle(comm_nh, "five"), "five_camera"),
  right_info_mgr(ros::NodeHandle(comm_nh, "right"), "right_camera") {

  /* default config values */
  width = 640;
  height = 480;
  fps = 10;
  skip_frames = 0;
  frames_to_skip = 0;
  left_device = "/dev/video0";
  right_device = "/dev/video1";
  center_device = "/dev/video2";
  center2_device = "/dev/video4";
  five_device = "/dev/video3";
  frame = "camera";
  rotate_left = false;
  rotate_right = false;
  rotate_center = false;
  rotate_center2 = false;
  rotate_five = false;

  /* set up information managers */
  std::string left_url, right_url, center_url, center2_url, five_url;

  pnode.getParam("left/camera_info_url", left_url);
  pnode.getParam("right/camera_info_url", right_url);
  pnode.getParam("center/camera_info_url", center_url);
  pnode.getParam("center2/camera_info_url", center2_url);
  pnode.getParam("five/camera_info_url", five_url);

  left_info_mgr.loadCameraInfo(left_url);
  right_info_mgr.loadCameraInfo(right_url);
  center_info_mgr.loadCameraInfo(center_url);
  center2_info_mgr.loadCameraInfo(center2_url);
  five_info_mgr.loadCameraInfo(five_url);

  /* pull other configuration */
  pnode.getParam("left/device", left_device);
  pnode.getParam("right/device", right_device);
  pnode.getParam("center/device", center_device);
  pnode.getParam("center2/device", center2_device);
  pnode.getParam("five/device", five_device);

  pnode.getParam("fps", fps);
  pnode.getParam("skip_frames", skip_frames);

  pnode.getParam("left/rotate", rotate_left);
  pnode.getParam("right/rotate", rotate_right);
  pnode.getParam("center/rotate", rotate_center);
  pnode.getParam("center2/rotate", rotate_center2);
  pnode.getParam("five/rotate", rotate_five);

  pnode.getParam("width", width);
  pnode.getParam("height", height);

  pnode.getParam("frame_id", frame);

  /* advertise image streams and info streams */
  left_pub = it.advertise("left/image_raw", 1);
  right_pub = it.advertise("right/image_raw", 1);
  center_pub = it.advertise("center/image_raw", 1);
  center2_pub = it.advertise("center2/image_raw", 1);
  five_pub = it.advertise("five/image_raw", 1);

  left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
  right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);
  center_info_pub = node.advertise<CameraInfo>("center/camera_info", 1);
  center2_info_pub = node.advertise<CameraInfo>("center2/camera_info", 1);
  five_info_pub = node.advertise<CameraInfo>("five/camera_info", 1);

  /* initialize the cameras */
  cam_left =
      new uvc_cam::Cam(left_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width, height, fps);
  cam_left->set_motion_thresholds(100, -1);
  cam_right =
      new uvc_cam::Cam(right_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width, height, fps);
  cam_right->set_motion_thresholds(100, -1);
  cam_center =
      new uvc_cam::Cam(center_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width, height, fps);
  cam_center->set_motion_thresholds(100, -1);
  cam_center2 =
      new uvc_cam::Cam(center2_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width, height, fps);
  cam_center2->set_motion_thresholds(100, -1);
  cam_five =
      new uvc_cam::Cam(five_device.c_str(), uvc_cam::Cam::MODE_RGB,
		       width, height, fps);
  cam_five->set_motion_thresholds(100, -1);



  bool auto_focus;
  if (pnode.getParam("auto_focus", auto_focus)) {
    cam_left->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
    cam_right->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
    cam_center->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
    cam_center2->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
    cam_five->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
  }

  int focus_absolute;
  if (pnode.getParam("focus_absolute", focus_absolute)) {
    cam_left->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
    cam_right->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
    cam_center->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
    cam_center2->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
    cam_five->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
  }

  bool auto_exposure;
  if (pnode.getParam("auto_exposure", auto_exposure)) {
    int val;
    if (auto_exposure) {
      val = V4L2_EXPOSURE_AUTO;
    } else {
      val = V4L2_EXPOSURE_MANUAL;
    }
    cam_left->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
    cam_right->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
    cam_center->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
    cam_center2->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
    cam_five->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
  }

  int exposure_absolute;
  if (pnode.getParam("exposure_absolute", exposure_absolute)) {
    cam_left->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
    cam_right->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
    cam_center->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
    cam_center2->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
    cam_five->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
  }

  int brightness;
  if (pnode.getParam("brightness", brightness)) {
    cam_left->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
    cam_right->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
    cam_center->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
    cam_five->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
  }

  int power_line_frequency;
  if (pnode.getParam("power_line_frequency", power_line_frequency)) {
    int val;
    if (power_line_frequency == 0) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
    } else if (power_line_frequency == 50) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
    } else if (power_line_frequency == 60) {
      val = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
    } else {
      printf("power_line_frequency=%d not supported. Using auto.\n", power_line_frequency);
      val = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
    }
    cam_left->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
    cam_right->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
    cam_center->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
    cam_center2->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
    cam_five->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
  }


  // TODO:
  // - add params for
  //   contrast
  //   saturation
  //   hue
  //   white balance temperature, auto and manual
  //   gamma
  //   sharpness
  //   backlight compensation
  //   exposure auto priority
  //   zoom
  // - add generic parameter list:
  //   [(id0, val0, name0), (id1, val1, name1), ...]


  /* and turn on the streamer */
  ok = true;
  image_thread = boost::thread(boost::bind(&StereoCamera::feedImages, this));
}

void StereoCamera::sendInfo(ros::Time time) {
  CameraInfoPtr info_left(new CameraInfo(left_info_mgr.getCameraInfo()));
  CameraInfoPtr info_right(new CameraInfo(right_info_mgr.getCameraInfo()));
  CameraInfoPtr info_center(new CameraInfo(center_info_mgr.getCameraInfo()));
  CameraInfoPtr info_center2(new CameraInfo(center2_info_mgr.getCameraInfo()));
  CameraInfoPtr info_five(new CameraInfo(five_info_mgr.getCameraInfo()));

  info_left->header.stamp = time;
  info_right->header.stamp = time;
  info_center->header.stamp = time;
  info_center2->header.stamp = time;
  info_five->header.stamp = time;
  info_left->header.frame_id = frame;
  info_right->header.frame_id = frame;
  info_center->header.frame_id = frame;
  info_center2->header.frame_id = frame;
  info_five->header.frame_id = frame;

  left_info_pub.publish(info_left);
  right_info_pub.publish(info_right);
  center_info_pub.publish(info_center);
  center2_info_pub.publish(info_center2);
  five_info_pub.publish(info_five);
}


void StereoCamera::feedImages() {
  unsigned int pair_id = 0;
  while (ok) {
    unsigned char *frame_left = NULL, *frame_right = NULL, *frame_center = NULL, *frame_center2 = NULL, *frame_five = NULL;
    uint32_t bytes_used_left, bytes_used_right, bytes_used_center, bytes_used_center2, bytes_used_five;

    ros::Time capture_time = ros::Time::now();

    int left_idx = cam_left->grab(&frame_left, bytes_used_left);
    int right_idx = cam_right->grab(&frame_right, bytes_used_right);
    int center_idx = cam_center->grab(&frame_center, bytes_used_center);
    int center2_idx = cam_center2->grab(&frame_center2, bytes_used_center2);
    int five_idx = cam_five->grab(&frame_five, bytes_used_five);

    /* Read in every frame the camera generates, but only send each
     * (skip_frames + 1)th frame. This reduces effective frame
     * rate, processing time and network usage while keeping the
     * images synchronized within the true framerate.
     */
    if (skip_frames == 0 || frames_to_skip == 0) {
      if (frame_left && frame_right && frame_center && frame_center2 && frame_five) {
	ImagePtr image_left(new Image);
	ImagePtr image_right(new Image);
	ImagePtr image_center(new Image);
	ImagePtr image_center2(new Image);
	ImagePtr image_five(new Image);

	image_left->height = height;
	image_left->width = width;
	image_left->step = 3 * width;
	image_left->encoding = image_encodings::RGB8;
	image_left->header.stamp = capture_time;
	image_left->header.seq = pair_id;

	image_right->height = height;
	image_right->width = width;
	image_right->step = 3 * width;
	image_right->encoding = image_encodings::RGB8;
	image_right->header.stamp = capture_time;
	image_right->header.seq = pair_id;

	image_center->height = height;
	image_center->width = width;
	image_center->step = 3 * width;
	image_center->encoding = image_encodings::RGB8;
	image_center->header.stamp = capture_time;
	image_center->header.seq = pair_id;

	image_center2->height = height;
	image_center2->width = width;
	image_center2->step = 3 * width;
	image_center2->encoding = image_encodings::RGB8;
	image_center2->header.stamp = capture_time;
	image_center2->header.seq = pair_id;

	image_five->height = height;
	image_five->width = width;
	image_five->step = 3 * width;
	image_five->encoding = image_encodings::RGB8;
	image_five->header.stamp = capture_time;
	image_five->header.seq = pair_id;

	image_left->header.frame_id = frame;
	image_right->header.frame_id = frame;
	image_center->header.frame_id = frame;
	image_center2->header.frame_id = frame;
	image_five->header.frame_id = frame;

	image_left->data.resize(image_left->step * image_left->height);
	image_right->data.resize(image_right->step * image_right->height);
	image_center->data.resize(image_center->step * image_center->height);
	image_center2->data.resize(image_center2->step * image_center2->height);
	image_five->data.resize(image_five->step * image_five->height);

	if (rotate_left)
	  rotate(&image_left->data[0], frame_left, width * height);
	else
	  memcpy(&image_left->data[0], frame_left, width * height * 3);

	if (rotate_right)
	  rotate(&image_right->data[0], frame_right, width * height);
	else
	  memcpy(&image_right->data[0], frame_right, width * height * 3);

	if (rotate_center)
	  rotate(&image_center->data[0], frame_center, width * height);
	else
	  memcpy(&image_center->data[0], frame_center, width * height * 3);

	if (rotate_center2)
	  rotate(&image_center2->data[0], frame_center2, width * height);
	else
	  memcpy(&image_center2->data[0], frame_center2, width * height * 3);

	if (rotate_five)
	  rotate(&image_five->data[0], frame_five, width * height);
	else
	  memcpy(&image_five->data[0], frame_five, width * height * 3);


	left_pub.publish(image_left);
	right_pub.publish(image_right);
	center_pub.publish(image_center);
	center2_pub.publish(image_center2);
	five_pub.publish(image_five);

	sendInfo(capture_time);

	++pair_id;
      }

      frames_to_skip = skip_frames;
    } else {
      frames_to_skip--;
    }

    if (frame_left)
      cam_left->release(left_idx);
    if (frame_right)
      cam_right->release(right_idx);
    if (frame_center)
      cam_center->release(center_idx);
    if (frame_center2)
      cam_center2->release(center2_idx);
    if (frame_five)
      cam_five->release(five_idx);
  }
}

StereoCamera::~StereoCamera() {
  ok = false;
  image_thread.join();
  if (cam_left)
    delete cam_left;
  if (cam_right)
    delete cam_right;
  if (cam_center)
    delete cam_center;
  if (cam_center2)
    delete cam_center2;
  if (cam_five)
    delete cam_five;
}

};
