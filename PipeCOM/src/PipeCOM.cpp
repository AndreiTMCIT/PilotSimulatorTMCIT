#include <iostream>
#include <array>
#include <map>
#include <string>  
#include <sstream>  
#include <iomanip>
#include <cmath>
#include <fstream>

#include <Windows.h>

#include <k4a/k4a.h>
#include <k4abt.h>

constexpr int SUCCESS = 0;
constexpr int FAILURE = 1;

#define VERIFY(result)		\
	if (result == FAILURE)	\
	{						\
		goto Exit;			\
	}

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

enum Joints {
	PELVIS, SPINE_NAVAL, SPINE_CHEST, NECK, CLAVICLE_LEFT, SHOULDER_LEFT,
	ELBOW_LEFT, WRIST_LEFT, HAND_LEFT, HANDTIP_LEFT, THUMB_LEFT, CLAVICLE_RIGHT,
	SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT, HAND_RIGHT, HANDTIP_RIGHT, THUMB_RIGHT,
	HIP_LEFT, KNEE_LEFT, ANKLE_LEFT, TOE_LEFT, HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT,
	TOE_RIGHT, HEAD, NOSE, EYE_LEFT, EAR_LEFT, EYE_RIGHT, EAR_RIGHT
};

enum BodySegments {
	FOOT_RIGHT, SHANK_RIGHT, THIGH_RIGHT, TRUNK_RIGHT,
	FOOT_LEFT, SHANK_LEFT, THIGH_LEFT, TRUNK_LEFT, HEAD_SEGMENT,
	BODY_SEGMENT_NUM
};

std::map<int, std::array<int, 2>> JointsToBodySegments{
	{ TOE_RIGHT, {FOOT_RIGHT} }, { ANKLE_RIGHT, {FOOT_RIGHT, SHANK_RIGHT} }, { KNEE_RIGHT, {SHANK_RIGHT, THIGH_RIGHT} },
	{ HIP_RIGHT, {THIGH_RIGHT, TRUNK_RIGHT} }, { SHOULDER_RIGHT, {TRUNK_RIGHT} },
	{ TOE_LEFT, {FOOT_LEFT} }, { ANKLE_LEFT, {FOOT_LEFT, SHANK_LEFT} }, { KNEE_LEFT, {SHANK_LEFT, THIGH_LEFT} },
	{ HIP_LEFT, {THIGH_LEFT, TRUNK_LEFT} }, { SHOULDER_LEFT, {TRUNK_LEFT} },
	{ NOSE, {HEAD_SEGMENT} }
};

std::map<int, std::array<int, 2>> JointsForBodySegments{
	{ FOOT_RIGHT, {TOE_RIGHT, ANKLE_RIGHT} }, { SHANK_RIGHT, {ANKLE_RIGHT, KNEE_RIGHT} }, { THIGH_RIGHT, {KNEE_RIGHT, HIP_RIGHT} },
	{ TRUNK_RIGHT, {HIP_RIGHT, SHOULDER_RIGHT} }, { FOOT_LEFT, {TOE_LEFT, ANKLE_LEFT} }, { SHANK_LEFT, {ANKLE_LEFT, KNEE_LEFT} }, 
	{ THIGH_LEFT, {KNEE_LEFT, HIP_LEFT} }, { TRUNK_LEFT, {HIP_LEFT, SHOULDER_LEFT} }, { HEAD_SEGMENT, {HEAD} }
};

enum BodySegmentsDataName {
	LENGTH, WEIGHT
};

std::map<int, std::array<float, 2>> BodySegmentsData{
	{ FOOT_RIGHT, {0.5, 0.0133} }, { SHANK_RIGHT, {0.419, 0.0535} }, 
	{ THIGH_RIGHT, {0.428, 0.1175} }, { TRUNK_RIGHT, {0.5, 0.225} },
	{ FOOT_LEFT, {0.5, 0.0133} }, { SHANK_LEFT, {0.419, 0.0535} }, 
	{ THIGH_LEFT, {0.428, 0.1175} }, { TRUNK_LEFT, {0.5, 0.225} },
	{ HEAD_SEGMENT, {0, 0.1814} }
};

void get_body_segment_com(k4a_float3_t body_segment_com[], bool body_segment_exist[], k4abt_joint_t joints[])
{
	k4a_float3_t proximal = {}, distal = {};
	float length = NULL;

	for (int segment_id = 0; segment_id < BODY_SEGMENT_NUM; segment_id++) {
		if (body_segment_exist[segment_id]) 
		{
			if (segment_id == HEAD_SEGMENT) 
			{
				body_segment_com[HEAD_SEGMENT].xyz = joints[NOSE].position.xyz;
			}
			else
			{
				length = BodySegmentsData[segment_id][LENGTH];
				
				proximal.xyz = joints[JointsForBodySegments[segment_id][1]].position.xyz;
				distal.xyz = joints[JointsForBodySegments[segment_id][0]].position.xyz;

				body_segment_com[segment_id].xyz.x = proximal.xyz.x + length * (distal.xyz.x - proximal.xyz.x);
				body_segment_com[segment_id].xyz.y = proximal.xyz.y + length * (distal.xyz.y - proximal.xyz.y);
				body_segment_com[segment_id].xyz.z = proximal.xyz.z + length * (distal.xyz.z - proximal.xyz.z);
			}	
		}
	}
}

void get_com(k4a_float3_t& center_of_mass, bool body_segment_exist[], k4a_float3_t body_segment_com[], k4abt_joint_t joints[])
{
	float weight = 0;

	get_body_segment_com(body_segment_com, body_segment_exist, joints);

	center_of_mass.xyz = { 0, 0, 0 };

	for (int segment_id = 0; segment_id < BODY_SEGMENT_NUM; segment_id++) {
		if (body_segment_exist[segment_id])
		{
			weight = BodySegmentsData[segment_id][WEIGHT];
			center_of_mass.xyz.x += body_segment_com[segment_id].xyz.x * weight;
			center_of_mass.xyz.y += body_segment_com[segment_id].xyz.y * weight;
			center_of_mass.xyz.z += body_segment_com[segment_id].xyz.z * weight;
		}
	}

	//std::cout << "X: " << center_of_mass.xyz.x << std::endl;
	//std::cout << "Y: " << center_of_mass.xyz.y << std::endl;
	//std::cout << "Z: " << center_of_mass.xyz.z << std::endl;

}

void start_com_tracking(k4a_device_t& device, k4a_calibration_t& device_calibration, k4abt_tracker_t& tracker, k4a_capture_t& capture)
{
	k4a_wait_result_t queue_capture_result = K4A_WAIT_RESULT_FAILED;
	k4abt_frame_t body_frame = NULL;
	k4a_wait_result_t pop_frame_result = K4A_WAIT_RESULT_FAILED;
	bool segment_exists[BODY_SEGMENT_NUM] = {};
	k4a_float3_t old_center_of_mass_3d = {0, 0, 0};
	k4a_float3_t center_of_mass_3d = {};
	k4a_float3_t com_difference = {};
	k4a_image_t color_image = NULL;
	uint8_t* color_image_buffer = NULL;
	k4abt_skeleton_t skeleton = {};
	k4a_float3_t body_segment_com[BODY_SEGMENT_NUM] = {};
	boolean reference_point_set = false;

	std::cout << "COM Tracking Start!" << std::endl;

	for (int i = 0; i < BODY_SEGMENT_NUM; i++) {
		segment_exists[i] = true;
	}

	std::fstream fs;
	fs.open("com_data.csv", std::ios::out, std::ios::trunc);

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

		color_image = k4a_capture_get_color_image(capture);

		int image_width = k4a_image_get_width_pixels(color_image);
		int image_height = k4a_image_get_height_pixels(color_image);

		color_image_buffer = k4a_image_get_buffer(color_image);

		k4abt_frame_get_body_skeleton(body_frame, 0, &skeleton);

		get_com(center_of_mass_3d, segment_exists, body_segment_com, skeleton.joints);

		com_difference.xyz.x = -(old_center_of_mass_3d.xyz.x - center_of_mass_3d.xyz.x);
		com_difference.xyz.y = -(old_center_of_mass_3d.xyz.y - center_of_mass_3d.xyz.y);
		com_difference.xyz.z = old_center_of_mass_3d.xyz.z - center_of_mass_3d.xyz.z;

		k4abt_frame_release(body_frame);
		k4a_capture_release(capture);

		if (GetKeyState(VK_ESCAPE) & 0x8000) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			fs.close();
			return;
		}

		if (GetKeyState(VK_SPACE) & 0x8000) { // Set reference point
			old_center_of_mass_3d.xyz = center_of_mass_3d.xyz;
			reference_point_set = true;
		}

		std::cout << std::endl;
		std::cout << "X: " << com_difference.xyz.x << std::endl;
		std::cout << "Y: " << com_difference.xyz.y << std::endl;
		std::cout << "Z: " << com_difference.xyz.z << std::endl;

		if (reference_point_set) {
			fs << com_difference.xyz.x << "," << com_difference.xyz.y << ","
				<< com_difference.xyz.z << "," << std::endl;
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
	k4abt_tracker_t* tracker
)
{

	if (*tracker != NULL)
	{
		k4abt_tracker_shutdown(*tracker);
		k4abt_tracker_destroy(*tracker);
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

int main (void)
{
	k4a_device_t device = NULL;
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_calibration_t calibration = {};
	k4a_transformation_t transformation = NULL;
	k4abt_tracker_t tracker = NULL;
	k4a_capture_t capture = NULL;

	VERIFY(get_device(device)); 
	VERIFY(start_camera(device, device_config));
	VERIFY(get_capture(device, capture));
	VERIFY(get_calibration(device, device_config, calibration));
	VERIFY(get_tracker(tracker, calibration));

	start_com_tracking(device, calibration, tracker, capture);
	
Exit:
	clear_memory(&device, &capture, &tracker);
	std::cout << "Exiting..." << std::endl;

	return SUCCESS;
}
