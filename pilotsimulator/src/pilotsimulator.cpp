#include "pilotsimulator.h"

namespace pilotsimulator {

	int get_device(k4a_device_t& device)
	{
		// Get devices count
		uint32_t count = k4a_device_get_installed_count();
		if (count == 0)
		{
			std::cout << "No Device Found!" << std::endl;
			return FAILURE;
		}
		else {
			std::cout << count << " Device(s) Found!" << std::endl;
		}

		// Open the first plugged in Kinect device
		if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device)))
		{
			std::cout << "Failed To Open Device!\n";
			return FAILURE;
		}

		// Get the size of the serial number
		size_t serial_size = 0;
		k4a_device_get_serialnum(device, NULL, &serial_size);

		// Allocate memory for the serial, then acquire it
		char* serial = (char*)(malloc(serial_size));
		k4a_device_get_serialnum(device, serial, &serial_size);
		std::cout << "Opened Device: " << serial << std::endl << std::endl;
		free(serial);

		return SUCCESS;
	}

	int start_camera(const k4a_device_t& device, k4a_device_configuration_t& device_config)
	{
		device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		device_config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
		device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		device_config.synchronized_images_only = true;

		if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &device_config))
		{
			std::cout << "Failed To Start Device!" << std::endl;
			return FAILURE;
		}

		return SUCCESS;
	}

	int get_capture(const k4a_device_t& device, k4a_capture_t& capture)
	{
		const int32_t TIMEOUT_IN_MS = 1000;
		switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
		{
		case K4A_WAIT_RESULT_SUCCEEDED:
			{
				std::cout << "Succesfully Read A Capture!\n" << std::endl;
				return SUCCESS;
			}
			break;
		case K4A_WAIT_RESULT_TIMEOUT:
			{
				std::cout << "Timed Out Waiting For A Capture!" << std::endl;
				return FAILURE;
			}
			break;
		case K4A_WAIT_RESULT_FAILED:
			{
				std::cout << "Failed To Read A Capture!" << std::endl;
				return FAILURE;
			}
			break;
		default:
			{
				return FAILURE;
			}
			break;
		}

		return FAILURE;
	}

	int get_calibration(
		const k4a_device_t& device, 
		const k4a_device_configuration_t& device_config, 
		k4a_calibration_t& calibration
	)
	{
		int result = k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &calibration);

		if (result != K4A_RESULT_SUCCEEDED)
		{
			std::cout << "Failed to get calibration" << std::endl;
			return FAILURE;
		}

		return SUCCESS;
	}

	int get_transformation(k4a_transformation_t& transformation, const k4a_calibration_t& calibration)
	{
		transformation = k4a_transformation_create(&calibration);

		if (transformation == NULL)
		{
			std::cout << "Failed to get transformation" << std::endl;
			return FAILURE;
		}

		return SUCCESS;
	}

	int get_tracker(k4abt_tracker_t& tracker, const k4a_calibration_t& calibration)
	{
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

		std::cout << "Creating Tracker..." << std::endl;

		int result = k4abt_tracker_create(&calibration, tracker_config, &tracker);

		if (result == K4A_RESULT_FAILED)
		{
			std::cout << "Failed to create tracker." << std::endl;
			return FAILURE;
		}

		std::cout << "Created Tracker!" << std::endl << std::endl;

		return SUCCESS;
	}

	void get_color_image(k4a_image_t& result_image, const k4a_capture_t& capture) 
	{
		result_image = k4a_capture_get_color_image(capture);

		return;
	}

	void get_depth_image(k4a_image_t& result_image, const k4a_capture_t& capture)
	{
		result_image = k4a_capture_get_depth_image(capture);

		return;
	}

	void get_color_in_depth_space_image(k4a_image_t& result_image, const k4a_capture_t& capture, k4a_transformation_t& transformation) 
	{
		result_image = NULL;

		k4a_image_t color_image = NULL;
		get_color_image(color_image, capture);

		k4a_image_t depth_image = NULL;
		get_depth_image(depth_image, capture);

		int depth_image_width = k4a_image_get_width_pixels(depth_image);
		int depth_image_height = k4a_image_get_height_pixels(depth_image);

		k4a_image_create(
			K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depth_image_width,
			depth_image_height,
			depth_image_width * 4 * (int)sizeof(uint8_t),
			&result_image
		);

		k4a_transformation_color_image_to_depth_camera(
			transformation,
			depth_image,
			color_image,
			result_image
		);

		return;
	}

	void get_depth_in_color_space_image(k4a_image_t& result_image, const k4a_capture_t& capture, k4a_transformation_t& transformation)
	{
		return;
	}

	void get_image(
		const Image image_type, 
		k4a_image_t images[], 
		const k4a_capture_t& capture, 
		k4a_transformation_t* transformation
	) {

		k4a_image_t* result_image = &images[image_type];

		switch (image_type) 
		{
		case COLOR:
			get_color_image(*result_image, capture);
			break;
		case DEPTH:
			get_depth_image(*result_image, capture);
			break;
		case COLOR_IN_DEPTH_SPACE:
			get_color_in_depth_space_image(*result_image, capture, *transformation);
			break;
		case DEPTH_IN_COLOR_SPACE:
			get_depth_in_color_space_image(*result_image, capture, *transformation);
			break;
		default:
			std::cout << "Wrong image type." << std::endl;
			break;
		}

		if (*result_image == NULL)
		{
			std::cout << "Failed to get image " << image_type << "." << std::endl;
			return;
		}

		return;
	}

	void get_body_image(k4a_image_t& result_image, const k4a_capture_t& capture, const k4abt_tracker_t& tracker)
	{
		k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);

		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);

		result_image = k4abt_frame_get_body_index_map(body_frame);

		k4abt_frame_release(body_frame);

		return;
	}

	void get_body_in_color_space_image(k4a_image_t& result_image, const k4a_capture_t& capture, const k4abt_tracker_t& tracker, const k4a_transformation_t& transformation)
	{
		k4a_image_t body_image = NULL;
		get_body_image(body_image, capture, tracker);

		k4a_image_t color_image = NULL;
		get_color_image(color_image, capture);

		k4a_image_t depth_image = NULL;
		get_depth_image(depth_image, capture);

		int color_image_width = k4a_image_get_width_pixels(color_image);
		int color_image_height = k4a_image_get_height_pixels(color_image);

		k4a_image_t depth_in_color_space_image = NULL;
		k4a_image_create(
			K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width,
			color_image_height,
			color_image_width * (int)sizeof(uint16_t),
			&depth_in_color_space_image
		);

		k4a_image_create(
			K4A_IMAGE_FORMAT_CUSTOM8,
			color_image_width,
			color_image_height,
			color_image_width * (int)sizeof(uint8_t),
			&result_image
		);

		k4a_transformation_depth_image_to_color_camera_custom(
			transformation,
			depth_image,
			body_image,
			depth_in_color_space_image,
			result_image,
			K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
			K4ABT_BODY_INDEX_MAP_BACKGROUND
		);

		return;
	}

	void get_body_color_overlay_image(const k4a_capture_t& capture, const k4abt_tracker_t& tracker, const k4a_transformation_t& transformation)
	{
		k4a_image_t color_image = NULL;
		get_color_image(color_image, capture);

		k4a_image_t body_in_color_space_image = NULL;
		get_body_in_color_space_image(body_in_color_space_image, capture, tracker, transformation);

		int image_height = k4a_image_get_height_pixels(color_image);
		int image_width = k4a_image_get_width_pixels(color_image);

		uint8_t* color_image_buffer = k4a_image_get_buffer(color_image);
		cv::Mat color_image_mat(
			image_height, image_width, CV_8UC4,
			(void*)color_image_buffer,
			cv::Mat::AUTO_STEP
		);

		uint8_t* body_in_color_space_image_buffer = k4a_image_get_buffer(body_in_color_space_image);
		cv::Mat body_in_color_space_image_mat(
			image_height, image_width, CV_8U,
			(void*)body_in_color_space_image_buffer,
			cv::Mat::AUTO_STEP
		);

		cv::Mat result_image_mat;
		color_image_mat.copyTo(result_image_mat);

		for (size_t row = 0; row < body_in_color_space_image_mat.rows; ++row)
		{
			uchar* currentPixel = body_in_color_space_image_mat.ptr<uchar>(row);
			for (size_t col = 0; col < body_in_color_space_image_mat.cols; ++col)
			{
				if ((int)*currentPixel++ == 255) continue; // ignore background

				result_image_mat.ptr<cv::Vec4b>(row)[col][0] = 255;
			}
		}

		cv::imwrite("body_color_overlay.jpg", result_image_mat);
	}

	void draw_skeleton(cv::Mat result_image_mat, boolean joints_exist[], k4a_float2_t joint_in_color_2d[(int)K4ABT_JOINT_COUNT]) {
		int joint_line[22][2] = {
			{PELVIS, SPINE_NAVAL}, {SPINE_NAVAL, SPINE_CHEST}, {SPINE_CHEST, NECK}, {NECK, CLAVICLE_LEFT},
			{CLAVICLE_LEFT, SHOULDER_LEFT}, {SHOULDER_LEFT, ELBOW_LEFT}, {ELBOW_LEFT, WRIST_LEFT}, {WRIST_LEFT, HAND_LEFT},
			{NECK, CLAVICLE_RIGHT}, {CLAVICLE_RIGHT, SHOULDER_RIGHT}, {SHOULDER_RIGHT, ELBOW_RIGHT}, {ELBOW_RIGHT, WRIST_RIGHT}, 
			{WRIST_RIGHT, HAND_RIGHT}, {PELVIS, HIP_LEFT}, {HIP_LEFT, KNEE_LEFT}, {KNEE_LEFT, ANKLE_LEFT}, {ANKLE_LEFT, TOE_LEFT},
			{PELVIS, HIP_RIGHT}, {HIP_RIGHT, KNEE_RIGHT}, {KNEE_RIGHT, ANKLE_RIGHT}, {ANKLE_RIGHT, TOE_RIGHT}, {NECK, HEAD}
		};

		for (int line_num = 0; line_num < 22; line_num++) {
			if (joints_exist[joint_line[line_num][0]] && joints_exist[joint_line[line_num][1]])
			{
				cv::Point point0 = cv::Point(joint_in_color_2d[joint_line[line_num][0]].v[0], joint_in_color_2d[joint_line[line_num][0]].v[1]);
				cv::Point point1 = cv::Point(joint_in_color_2d[joint_line[line_num][1]].v[0], joint_in_color_2d[joint_line[line_num][1]].v[1]);

				cv::line(
					result_image_mat,
					point0,
					point1,
					cv::Scalar(255, 255, 255),
					10,
					cv::LINE_8,
					0
				);
			}
		}
	}

	void get_body_segment_com(k4abt_skeleton_t& skeleton, boolean joint_exists[], k4a_float3_t body_segment_com[])
	{
		if (joint_exists[ANKLE_RIGHT] && joint_exists[TOE_RIGHT]) {

			int proximal_name = ANKLE_RIGHT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = TOE_RIGHT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = FOOT_RIGHT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.5f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.5f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.5f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[KNEE_RIGHT] && joint_exists[ANKLE_RIGHT]) {

			int proximal_name = KNEE_RIGHT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = ANKLE_RIGHT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = SHANK_RIGHT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.419f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.419f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.419f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[KNEE_RIGHT] && joint_exists[HIP_RIGHT]) {

			int proximal_name = KNEE_RIGHT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = HIP_RIGHT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = THIGH_RIGHT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.428f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.428f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.428f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[HIP_RIGHT] && joint_exists[SHOULDER_RIGHT]) {

			int proximal_name = HIP_RIGHT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = SHOULDER_RIGHT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = TRUNK_RIGHT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.5f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.5f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.5f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[ANKLE_LEFT] && joint_exists[TOE_LEFT]) {

			int proximal_name = ANKLE_LEFT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = TOE_LEFT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = FOOT_LEFT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.5f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.5f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.5f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[KNEE_LEFT] && joint_exists[ANKLE_LEFT]) {

			int proximal_name = KNEE_LEFT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = ANKLE_LEFT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = SHANK_LEFT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.419f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.419f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.419f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[KNEE_LEFT] && joint_exists[HIP_LEFT]) {

			int proximal_name = KNEE_LEFT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = HIP_LEFT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = THIGH_LEFT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.428f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.428f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.428f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		if (joint_exists[HIP_LEFT] && joint_exists[SHOULDER_LEFT]) {

			int proximal_name = HIP_LEFT;
			k4a_float3_t proximal_point = {};
			proximal_point.xyz.x = skeleton.joints[proximal_name].position.xyz.x;
			proximal_point.xyz.y = skeleton.joints[proximal_name].position.xyz.y;
			proximal_point.xyz.z = skeleton.joints[proximal_name].position.xyz.z;

			int distal_name = SHOULDER_LEFT;
			k4a_float3_t distal_point = {};
			distal_point.xyz.x = skeleton.joints[distal_name].position.xyz.x;
			distal_point.xyz.y = skeleton.joints[distal_name].position.xyz.y;
			distal_point.xyz.z = skeleton.joints[distal_name].position.xyz.z;

			int segment_name = TRUNK_LEFT;
			body_segment_com[segment_name].xyz.x = proximal_point.xyz.x + 0.5f * (distal_point.xyz.x - proximal_point.xyz.x);
			body_segment_com[segment_name].xyz.y = proximal_point.xyz.y + 0.5f * (distal_point.xyz.y - proximal_point.xyz.y);
			body_segment_com[segment_name].xyz.z = proximal_point.xyz.z + 0.5f * (distal_point.xyz.z - proximal_point.xyz.z);
		}

		std::cout << "GOT BODY SEGMENT" << std::endl;
	}

	void get_skeleton_in_color_space_image(const k4a_capture_t& capture, const k4abt_tracker_t& tracker, const k4a_calibration_t& calibration)
	{
		k4a_image_t color_image = NULL;
		get_color_image(color_image, capture);

		int image_height = k4a_image_get_height_pixels(color_image);
		int image_width = k4a_image_get_width_pixels(color_image);

		uint8_t* color_image_buffer = k4a_image_get_buffer(color_image);
		cv::Mat color_image_mat(
			image_height, image_width, CV_8UC4,
			(void*)color_image_buffer,
			cv::Mat::AUTO_STEP
		);

		cv::Mat result_image_mat;
		color_image_mat.copyTo(result_image_mat);

		k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);

		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);

		uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

		k4a_float2_t joint_in_color_2d[10][(int)K4ABT_JOINT_COUNT] = {};

		//// Transform each 3d joints from 3d depth space to 2d color image space
		for (uint32_t i = 0; i < num_bodies; i++)
		{

			k4abt_skeleton_t skeleton;
			k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

			boolean joints_exist[(int)K4ABT_JOINT_COUNT] = {};

			for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++)
			{
				int valid;

				k4a_calibration_3d_to_2d(
					&calibration,
					&skeleton.joints[joint_id].position,
					K4A_CALIBRATION_TYPE_DEPTH,
					K4A_CALIBRATION_TYPE_COLOR,
					&joint_in_color_2d[i][joint_id],
					&valid
				);

				if (valid && joint_id != NOSE && joint_id != EYE_LEFT && joint_id != EYE_RIGHT && joint_id != EAR_LEFT && joint_id != EAR_RIGHT && joint_id != HANDTIP_LEFT && joint_id != HANDTIP_RIGHT)
				{

					cv::Point joint_point = cv::Point(joint_in_color_2d[i][joint_id].v[0], joint_in_color_2d[i][joint_id].v[1]);
					cv::circle(
						result_image_mat,
						joint_point,
						20,
						cv::Scalar(255, 255, 255),
						cv::FILLED,
						8,
						0
					);

					joints_exist[joint_id] = TRUE;
				}
				else {
					joints_exist[joint_id] = FALSE;
				}
			}

			draw_skeleton(result_image_mat, joints_exist, joint_in_color_2d[i]);
		}

		cv::imwrite("skeleton_in_color_space.jpg", result_image_mat);

		k4abt_frame_release(body_frame);
	}

	void get_body_tracking_image(
		const Image image_type,
		k4a_image_t images[],
		const k4a_capture_t& capture,
		const k4abt_tracker_t& tracker,
		k4a_transformation_t* transformation,
		k4a_calibration_t* calibration
	) 
	{
		k4a_image_t* result_image = &images[image_type];

		switch (image_type)
		{
		case BODY:
			get_body_image(*result_image, capture, tracker);
			break;
		case BODY_IN_COLOR_SPACE:
			get_body_in_color_space_image(*result_image, capture, tracker, *transformation);
			break;
		case BODY_COLOR_OVERLAY:
			get_body_color_overlay_image(capture, tracker, *transformation);
			break;
		case SKELETON_IN_COLOR_SPACE:
			get_skeleton_in_color_space_image(capture, tracker, *calibration);
			break;
		default:
			std::cout << "Wrong image type." << std::endl;
			break;
		}

		if (*result_image == NULL)
		{
			std::cout << "Failed to get image " << image_type << std::endl;
			return;
		}

		return;
	}

	int get_cv_mat_type(Image image_type) {
		switch (image_type)
		{
		case COLOR:
		case COLOR_IN_DEPTH_SPACE:
		case DEPTH_IN_COLOR_SPACE:
		case BODY_COLOR_OVERLAY:
			return CV_8UC4;
		case DEPTH:
			return CV_16U;
		case BODY:
		case BODY_IN_COLOR_SPACE:
			return CV_8U;
		default:
			return 0;
		}
	}

	void save_image(const Image image_type, const k4a_image_t images[], std::string filename)
	{
		const k4a_image_t original_image = images[image_type];

		if (original_image == NULL) {
			std::cout << "Image is Null!" << std::endl;
			return;
		}

		int image_height = k4a_image_get_height_pixels(original_image);
		int image_width = k4a_image_get_width_pixels(original_image);

		uint8_t* image_buffer = k4a_image_get_buffer(original_image);

		cv::Mat image_mat(
			image_height, 
			image_width, 
			get_cv_mat_type(image_type), 
			(void*)image_buffer, 
			cv::Mat::AUTO_STEP
		);

		cv::imwrite(filename, image_mat);
		std::cout << "Image is stored at " << filename << std::endl;

		return;
	}

	void start_body_tracking(k4a_device_t& device, k4a_capture_t& capture, k4abt_tracker_t& tracker)
	{
		k4a_wait_result_t queue_capture_result = K4A_WAIT_RESULT_FAILED;
		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = K4A_WAIT_RESULT_FAILED;
		size_t num_bodies = NULL;

		std::cout << "Body Tracking Start!" << std::endl;

	BodyTracking:

		if (get_capture(device, capture) == FAILURE) { return; };

		queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
		if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
		{
			std::cout << "Failed to add capture to tracker process queue." << std::endl;
			return;
		}

		pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			num_bodies = k4abt_frame_get_num_bodies(body_frame);
			std::cout << "Body Tracked: " << k4abt_frame_get_num_bodies(body_frame) << std::endl;

			k4abt_frame_release(body_frame);
			k4a_capture_release(capture);

			if (GetKeyState(VK_ESCAPE) & 0x8000/*Check if high-order bit is set (1 << 15)*/)
			{
				return;
			}
		}
		else {
			std::cout << "Failed to pop capture from tracker process queue." << std::endl;
			return;
		}

		goto BodyTracking;
	}

	void stream_images(k4a_device_t& device, k4a_capture_t& capture, k4a_calibration_t& calibration,  k4abt_tracker_t& tracker)
	{
		k4a_wait_result_t queue_capture_result = K4A_WAIT_RESULT_FAILED;
		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = K4A_WAIT_RESULT_FAILED;
		size_t num_bodies = NULL;
		k4a_image_t color_image = NULL;
		k4a_image_t depth_image = NULL;
		k4a_image_t body_image = NULL;
		k4a_image_t body_in_color_space_image = NULL;
		k4a_image_t depth_in_color_space_image = NULL;
		cv::Mat body_color_overlay_image_mat;
		uint8_t* depth_image_buffer;
		uint8_t* color_image_buffer;
		uint8_t* body_in_color_space_image_buffer;
		k4a_float2_t joint_in_color_2d[10][(int)K4ABT_JOINT_COUNT] = {};

		k4a_transformation_t transformation = NULL;
		get_transformation(transformation, calibration);

		std::cout << "Streaming Images!" << std::endl;

	BodyTracking:

		if (get_capture(device, capture) == FAILURE) { return; };

		queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
		if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
		{
			std::cout << "Failed to add capture to tracker process queue." << std::endl;
			return;
		}

		pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			num_bodies = k4abt_frame_get_num_bodies(body_frame);
			std::cout << "Body Tracked: " << k4abt_frame_get_num_bodies(body_frame) << std::endl;

			get_color_image(color_image,  capture);
			get_depth_image(depth_image, capture);

			body_image = k4abt_frame_get_body_index_map(body_frame);

			int image_width = k4a_image_get_width_pixels(color_image);
			int image_height = k4a_image_get_height_pixels(color_image);

			int depth_image_width = k4a_image_get_width_pixels(color_image);
			int depth_image_height = k4a_image_get_height_pixels(color_image);

			k4a_image_create(
				K4A_IMAGE_FORMAT_DEPTH16,
				image_width,
				image_height,
				image_width * (int)sizeof(uint16_t),
				&depth_in_color_space_image
			);

			k4a_image_create(
				K4A_IMAGE_FORMAT_CUSTOM8,
				image_width,
				image_height,
				image_width * (int)sizeof(uint8_t),
				&body_in_color_space_image
			);

			k4a_transformation_depth_image_to_color_camera_custom(
				transformation,
				depth_image,
				body_image,
				depth_in_color_space_image,
				body_in_color_space_image,
				K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
				K4ABT_BODY_INDEX_MAP_BACKGROUND
			);

			depth_image_buffer = k4a_image_get_buffer(color_image);
			cv::Mat depth_image_mat(
				depth_image_height, depth_image_width, CV_16U,
				(void*)depth_image_buffer,
				cv::Mat::AUTO_STEP
			);

			color_image_buffer = k4a_image_get_buffer(color_image);
			cv::Mat color_image_mat(
				image_height, image_width, CV_8UC4,
				(void*)color_image_buffer,
				cv::Mat::AUTO_STEP
			);

			body_in_color_space_image_buffer = k4a_image_get_buffer(body_in_color_space_image);
			cv::Mat body_in_color_space_image_mat(
				image_height, image_width, CV_8U,
				(void*)body_in_color_space_image_buffer,
				cv::Mat::AUTO_STEP
			);

			color_image_mat.copyTo(body_color_overlay_image_mat);

			for (size_t row = 0; row < body_in_color_space_image_mat.rows; ++row)
			{
				uchar* currentPixel = body_in_color_space_image_mat.ptr<uchar>(row);
				for (size_t col = 0; col < body_in_color_space_image_mat.cols; ++col)
				{
					if ((int)*currentPixel++ == 255) continue; // ignore background

					body_color_overlay_image_mat.ptr<cv::Vec4b>(row)[col][0] = 255;
				}
			}

			//// Transform each 3d joints from 3d depth space to 2d color image space
			for (uint32_t i = 0; i < num_bodies; i++)
			{

				k4abt_skeleton_t skeleton;
				k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);

				boolean joints_exist[(int)K4ABT_JOINT_COUNT] = {};

				k4a_float3_t body_segment_com[BODY_SEGMENT_END] = {};
				k4a_float2_t segment_in_color_2d[BODY_SEGMENT_END] = {};

				for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++)
				{
					int valid;

					float joint_positions[3] = {
						skeleton.joints[joint_id].position.v[0],
						skeleton.joints[joint_id].position.v[1],
						skeleton.joints[joint_id].position.v[2]
					};

					std::cout <<
						"X: " <<
						joint_positions[0] <<
						"Y: " <<
						joint_positions[1] <<
						"Z: " <<
						joint_positions[2] <<
						std::endl;

					k4a_calibration_3d_to_2d(
						&calibration,
						&skeleton.joints[joint_id].position,
						K4A_CALIBRATION_TYPE_DEPTH,
						K4A_CALIBRATION_TYPE_COLOR,
						&joint_in_color_2d[i][joint_id],
						&valid
					);

					if (valid && joint_id != NOSE && joint_id != EYE_LEFT && joint_id != EYE_RIGHT && joint_id != EAR_LEFT && joint_id != EAR_RIGHT && joint_id != HANDTIP_LEFT && joint_id != HANDTIP_RIGHT)
					{

						cv::Point joint_point = cv::Point(joint_in_color_2d[i][joint_id].v[0], joint_in_color_2d[i][joint_id].v[1]);
						cv::circle(
							body_color_overlay_image_mat,
							joint_point,
							20,
							cv::Scalar(255, 255, 255),
							cv::FILLED,
							8,
							0
						);

						joints_exist[joint_id] = TRUE;
					}
					else {
						joints_exist[joint_id] = FALSE;
					}
				}

				std::cout << "GETTING BODY SEGMENTS" << std::endl;
				get_body_segment_com(skeleton, joints_exist, body_segment_com);

				int segment_num = 0;
				for (const k4a_float3_t& value : body_segment_com)
				{
					int valid_segment;

					std::cout << std::endl << std::endl <<
						"BODY-X:" << value.xyz.x << std::endl <<
						"BODY-Y:" << value.xyz.y << std::endl <<
						"BODY-Z:" << value.xyz.z << std::endl;

					k4a_calibration_3d_to_2d(
						&calibration,
						&value,
						K4A_CALIBRATION_TYPE_DEPTH,
						K4A_CALIBRATION_TYPE_COLOR,
						&segment_in_color_2d[segment_num],
						&valid_segment
					);

					if (valid_segment)
					{
						std::cout << "writing skeleton" << std::endl;

						cv::Point joint_point = cv::Point(segment_in_color_2d[segment_num].xy.x, segment_in_color_2d[segment_num].xy.y);
						cv::circle(
							body_color_overlay_image_mat,
							joint_point,
							20,
							cv::Scalar(0, 255, 0),
							cv::FILLED,
							8,
							0
						);
					}

					segment_num++;
				}

				draw_skeleton(body_color_overlay_image_mat, joints_exist, joint_in_color_2d[i]);
			}

			cv::imshow("color_image", color_image_mat);
			cv::imshow("depth_image", depth_image_mat);
			cv::imshow("body_color_overlay_image", body_color_overlay_image_mat);

			k4abt_frame_release(body_frame);
			k4a_capture_release(capture);

			if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				k4a_transformation_destroy(transformation);
				return;
			}
		}
		else {
			std::cout << "Failed to pop capture from tracker process queue." << std::endl;
			return;
		}

		goto BodyTracking;
	}

	void clear_memory(
		k4a_device_t* device, 
		k4a_capture_t* capture, 
		k4a_image_t images[], 
		k4a_transformation_t* transformation,
		k4abt_tracker_t* tracker
	)
	{

		if (*transformation != NULL) {
			k4a_transformation_destroy(*transformation);
		}

		if (*tracker != NULL)
		{
			k4abt_tracker_shutdown(*tracker);
			k4abt_tracker_destroy(*tracker);
		}

		for (int i = 0; i < TOTAL_IMAGE_NUMBER; i++) {
			if (images[i] != NULL)
			{
				k4a_image_release(images[i]);
			}
		}

		if (*capture != NULL)
		{
			k4a_capture_release(*capture);
		}

		if (*device != NULL)
		{
			k4a_device_stop_cameras(*device);
			k4a_device_close(*device);
		}

	}
}