#include <iostream>

#include "pilotsimulator.h"

#include <k4a/k4a.h>

using namespace pilotsimulator;

#define VERIFY(result)		\
	if (result == FAILURE)	\
	{						\
		goto Exit;			\
	}

int main(void)
{
	std::cout << "Running GetImages.cpp\n\n";

	k4a_device_t device = NULL;
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_capture_t capture = NULL;
	k4a_calibration_t calibration = {};
	k4abt_tracker_t tracker = NULL;
	k4a_transformation_t transformation = NULL;
	k4a_image_t images[pilotsimulator::TOTAL_IMAGE_NUMBER] = {};

	VERIFY(get_device(device));
	VERIFY(start_camera(device, device_config));
	VERIFY(get_capture(device, capture));
	VERIFY(get_calibration(device, device_config, calibration));
	VERIFY(get_tracker(tracker, calibration));
	VERIFY(get_transformation(transformation, calibration));

	get_body_tracking_image(BODY, images, capture, tracker);
	get_body_tracking_image(BODY_IN_COLOR_SPACE, images, capture, tracker, &transformation);

	save_image(BODY, images, "body.jpg");
	save_image(BODY_IN_COLOR_SPACE, images, "body_in_color_space.jpg");

	get_body_tracking_image(BODY_COLOR_OVERLAY, images, capture, tracker, &transformation);
	get_body_tracking_image(SKELETON_IN_COLOR_SPACE, images, capture, tracker, NULL, &calibration);

Exit:
	clear_memory(&device, &capture, images);

	return 0;
}