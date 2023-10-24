#pragma once

#include <iostream>

#include <Windows.h>

#include <k4a/k4a.h>
#include <k4abt.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace pilotsimulator {

	constexpr int SUCCESS = 0;
	constexpr int FAILURE = 1;

	enum Image {
		COLOR, DEPTH, COLOR_IN_DEPTH_SPACE,
		DEPTH_IN_COLOR_SPACE, BODY,
		BODY_IN_COLOR_SPACE, BODY_COLOR_OVERLAY,
		SKELETON_IN_COLOR_SPACE,
		ALWAYS_LAST
	};
	constexpr Image TOTAL_IMAGE_NUMBER = ALWAYS_LAST;

	enum Joints {
		PELVIS, SPINE_NAVAL, SPINE_CHEST, NECK, CLAVICLE_LEFT, SHOULDER_LEFT,
		ELBOW_LEFT, WRIST_LEFT, HAND_LEFT, HANDTIP_LEFT, THUMB_LEFT, CLAVICLE_RIGHT,
		SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT, HAND_RIGHT, HANDTIP_RIGHT, THUMB_RIGHT,
		HIP_LEFT, KNEE_LEFT, ANKLE_LEFT, TOE_LEFT, HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT,
		TOE_RIGHT, HEAD, NOSE, EYE_LEFT, EAR_LEFT, EYE_RIGHT, EAR_RIGHT
	};

	enum Body_Segments {
		FOOT_RIGHT, SHANK_RIGHT,THIGH_RIGHT, TRUNK_RIGHT, FOOT_LEFT, SHANK_LEFT, THIGH_LEFT, TRUNK_LEFT, BODY_SEGMENT_END 
	};

	// Get k4a device
	int get_device(k4a_device_t& device);

	int start_camera(
		const k4a_device_t& device,
		k4a_device_configuration_t& device_config
	);

	int get_capture(const k4a_device_t& device, k4a_capture_t& capture);

	int get_calibration(
		const k4a_device_t& device, 
		const k4a_device_configuration_t& device_config, 
		k4a_calibration_t& calibration
	);

	int get_transformation(k4a_transformation_t& transformation, const k4a_calibration_t& calibration);

	int get_tracker(k4abt_tracker_t& tracker, const k4a_calibration_t& calibration);

	void get_image(
		const Image image_type, 
		k4a_image_t images[], 
		const k4a_capture_t& capture, 
		k4a_transformation_t* transformation = NULL
	);

	void get_body_tracking_image(
		const Image image_type,
		k4a_image_t images[],
		const k4a_capture_t& capture,
		const k4abt_tracker_t& tracker,
		k4a_transformation_t* transformation = NULL,
		k4a_calibration_t* calibration = NULL
	);

	int get_cv_mat_type(Image image_type);

	void save_image(const Image image_type, const k4a_image_t images[], std::string filename);

	void start_body_tracking(k4a_device_t& device, k4a_capture_t& capture, k4abt_tracker_t& tracker);

	void stream_images(k4a_device_t& device, k4a_capture_t& capture, k4a_calibration_t& calibration, k4abt_tracker_t& tracker);

	void clear_memory(
		k4a_device_t* device = NULL, 
		k4a_capture_t* capture = NULL, 
		k4a_image_t images[] = {},
		k4a_transformation_t* transformation = NULL,
		k4abt_tracker_t* tracker = NULL
	);
}