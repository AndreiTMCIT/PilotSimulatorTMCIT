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
	std::cout << "Running: StreamImages.cpp" << std::endl << std::endl;

	k4a_device_t device = NULL;
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	k4a_capture_t capture = NULL;
	k4a_calibration_t calibration = {};
	k4abt_tracker_t tracker = NULL;

	VERIFY(get_device(device));
	VERIFY(start_camera(device, device_config));
	VERIFY(get_capture(device, capture));
	VERIFY(get_calibration(device, device_config, calibration));
	VERIFY(get_tracker(tracker, calibration));

	stream_images(device, capture, calibration, tracker);

Exit:
	clear_memory(&device, &capture, NULL, NULL, &tracker);

	return 0;
}