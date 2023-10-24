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
	k4a_transformation_t transformation = NULL;
	k4a_image_t images[pilotsimulator::TOTAL_IMAGE_NUMBER] = {};

	VERIFY(get_device(device));
	VERIFY(start_camera(device, device_config));
	VERIFY(get_capture(device, capture));
	VERIFY(get_calibration(device, device_config, calibration));
	VERIFY(get_transformation(transformation, calibration));

	get_image(COLOR, images, capture);
	get_image(DEPTH, images, capture);
	get_image(COLOR_IN_DEPTH_SPACE, images, capture, &transformation);

	save_image(COLOR, images, "color.jpg");
	save_image(DEPTH, images, "depth.jpg");
	save_image(COLOR_IN_DEPTH_SPACE, images, "color_in_depth_space.jpg");

Exit:
	clear_memory(&device, &capture, images);

	return 0;
}