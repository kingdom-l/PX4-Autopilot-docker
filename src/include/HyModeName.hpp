#pragma once

#include <uORB/topics/vehicle_status.h>

#define HYDRO_MODE_STABILIZED	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL1)
#define HYDRO_MODE_ACRO		(vehicle_status_s::NAVIGATION_STATE_EXTERNAL2)
#define HYDRO_MODE_MANUAL	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL3)
#define HYDRO_MODE_AUTO_DIVE	(vehicle_status_s::NAVIGATION_STATE_EXTERNAL4)

