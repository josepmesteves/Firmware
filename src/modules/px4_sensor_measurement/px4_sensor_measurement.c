/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_sensor_measurement.c
 * Minimal application to retrieve sensor measurements from PX4 board
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <drivers/drv_range_finder.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

__EXPORT int px4_sensor_measurement_main(int argc, char *argv[]);

int px4_sensor_measurement_main(int argc, char *argv[])
{
	bool first_iteration = true;
	
	/* subscribe to sensor_combined topic and limit sampling rate (rate of updates)*/
	int sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int sensor_sonar_sub_fd = orb_subscribe(ORB_ID(sensor_range_finder));
	
	orb_set_interval(sensor_combined_sub_fd, 10);
	orb_set_interval(sensor_sonar_sub_fd, 100);
	
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = sensor_combined_sub_fd,   .events = POLLIN },
		{ .fd = sensor_sonar_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
	
	int error_counter = 0;
	
	for (int i = 0; i < 1000; i++) {
		/* wait for sensor update of 1 file descriptor for 10 ms */
		int poll_ret = poll(fds, 2, 150);

			/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[px4_sensor_measurement] Got no data within 200ms\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_sensor_measurement] ERROR return value from poll(): %d\n"
					   , poll_ret);
			}
			error_counter++;
		} else {
			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct range_finder_report sonar;
				uint64_t previous_time_stamp;
				uint64_t dt;
				float frequency;

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_range_finder), sensor_sonar_sub_fd, &sonar);
				
				if(first_iteration){
					printf("first iteration.\n");
					previous_time_stamp = sonar.timestamp;
					first_iteration = false;
					continue;
				}
				
				dt = sonar.timestamp - previous_time_stamp; // micro seconds
				previous_time_stamp=sonar.timestamp;
				frequency = 1E6/dt; // Hz
				printf("distance= %2.3f \t time= %lld \t dt= %lld \t %8.3f\n", (double)sonar.distance, previous_time_stamp, dt, (double)frequency);
			}
			if (fds[0].revents & POLLIN) {
				
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &raw);
				printf("-----sensor: %f\n", (double)raw.magnetometer_raw[0]);
			}

		}
	}
	
	return 0;
}
