/****************************************************************************
 *
 *   Copyright (C) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file position_estimator_inav_main.c
 * Model-identification based position estimator for multirotors
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/optical_flow.h>
#include <drivers/drv_range_finder.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s
#define BARO_FILT_WIND_SIZE 35 // window size of the barometer moving average filter

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_inav_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

static const hrt_abstime vision_topic_timeout = 500000;	// Vision topic timeout = 0.5s
static const hrt_abstime gps_topic_timeout = 500000;		// GPS topic timeout = 0.5s
static const hrt_abstime flow_topic_timeout = 1000000;	// optical flow topic timeout = 1s
static const hrt_abstime sonar_timeout = 150000;	// sonar timeout = 150ms
static const hrt_abstime sonar_valid_timeout = 1000000;	// estimate sonar distance during this time after sonar loss
static const hrt_abstime xy_src_timeout = 2000000;	// estimate position during this time after position sources loss
static const uint32_t updates_counter_len = 1000000;
static const float max_flow = 1.0f;	// max flow value that can be used, rad/s

__EXPORT int position_estimator_inav_main(int argc, char *argv[]);

int position_estimator_inav_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [-v]\n\n");
	exit(1);
}

/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_inav_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		verbose_mode = false;

		if (argc > 1)
			if (!strcmp(argv[2], "-v")) {
				verbose_mode = true;
			}

		thread_should_exit = false;
		position_estimator_inav_task = task_spawn_cmd("position_estimator_inav",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 5000,
					       position_estimator_inav_thread_main,
					       (argv) ? (char * const *) &argv[2] : (char * const *) NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static void write_debug_log(const char *msg, float dt, float x_est[2], float y_est[2], float z_est[2], float x_est_prev[2], float y_est_prev[2], float z_est_prev[2], float acc[3], float corr_gps[3][2], float w_xy_gps_p, float w_xy_gps_v)
{
	FILE *f = fopen("/fs/microsd/inav.log", "a");

	if (f) {
		char *s = malloc(256);
		unsigned n = snprintf(s, 256, "%llu %s\n\tdt=%.5f x_est=[%.5f %.5f] y_est=[%.5f %.5f] z_est=[%.5f %.5f] x_est_prev=[%.5f %.5f] y_est_prev=[%.5f %.5f] z_est_prev=[%.5f %.5f]\n",
                              hrt_absolute_time(), msg, (double)dt,
                              (double)x_est[0], (double)x_est[1], (double)y_est[0], (double)y_est[1], (double)z_est[0], (double)z_est[1],
                              (double)x_est_prev[0], (double)x_est_prev[1], (double)y_est_prev[0], (double)y_est_prev[1], (double)z_est_prev[0], (double)z_est_prev[1]);
		fwrite(s, 1, n, f);
		n = snprintf(s, 256, "\tacc=[%.5f %.5f %.5f] gps_pos_corr=[%.5f %.5f %.5f] gps_vel_corr=[%.5f %.5f %.5f] w_xy_gps_p=%.5f w_xy_gps_v=%.5f\n",
                     (double)acc[0], (double)acc[1], (double)acc[2],
                     (double)corr_gps[0][0], (double)corr_gps[1][0], (double)corr_gps[2][0], (double)corr_gps[0][1], (double)corr_gps[1][1], (double)corr_gps[2][1],
                     (double)w_xy_gps_p, (double)w_xy_gps_v);
		fwrite(s, 1, n, f);
		free(s);
	}

	fsync(fileno(f));
	fclose(f);
}

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_inav_thread_main(int argc, char *argv[])
{
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel

	float est_buf[EST_BUF_SIZE][3][2];	// estimated position buffer
	float R_buf[EST_BUF_SIZE][3][3];	// rotation matrix buffer
	float R_gps[3][3];					// rotation matrix for GPS correction moment
	float baro_buf[BARO_FILT_WIND_SIZE];
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	memset(baro_buf, 0, sizeof(baro_buf));
	int buf_ptr = 0;
	int baro_ptr = 0;

//	static const float min_eph_epv = 2.0f;	// min EPH/EPV, used for weight calculation
	static const float max_eph_epv = 20.0f;	// max EPH/EPV acceptable for estimation

	float eph = max_eph_epv;
//	float epv = 1.0f;

	float eph_flow = 1.0f;

	float eph_vision = 0.2f;
//	float epv_vision = 0.2f;

	float x_est_prev[2], y_est_prev[2], z_est_prev[2];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

//	int acc_ctr = 0;
	int baro_init_cnt = 0;
	int baro_init_num = 200;
	float baro_offset = 0.0f;		// baro offset for reference altitude, initialized on start, then adjusted
/*
	float surface_offset = 0.0f;	// ground level offset from reference altitude
	float surface_offset_rate = 0.0f;	// surface offset change rate
*/

	hrt_abstime accel_timestamp = 0;
	hrt_abstime baro_timestamp = 0;

	bool ref_inited = false;
//	hrt_abstime ref_init_start = 0;
//	const hrt_abstime ref_init_delay = 1000000;	// wait for 1s after 3D fix
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));
	hrt_abstime home_timestamp = 0;

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;

	hrt_abstime updates_counter_start = hrt_absolute_time();
	hrt_abstime pub_last = hrt_absolute_time();

	hrt_abstime t_prev = 0;
	
	// TODO tune weight
	int sonar_weight = 0.5;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
	float corr_baro = 0.0f;		// D
	float corr_gps[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
	float w_gps_xy = 1.0f;
//	float w_gps_z = 1.0f;

	float corr_vision[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};

	float corr_sonar = 0.0f;
	float corr_sonar_filtered = 0.0f;
	float acceptable_change = 0.0f;

	float corr_flow[] = { 0.0f, 0.0f };	// N E
	float w_flow = 0.0f;

	float sonar_last_valid = 0.0f;
	float sonar_new = 0.0f;

	float baro_new = 0.0f;
	
	float last_filtered_acc = 0.0f;
	
	//hrt_abstime flow_prev = 0;			// time of last flow measurement
	hrt_abstime sonar_time = 0;			// time of last sonar measurement (not filtered)
	hrt_abstime sonar_time_prev = 0;	// time of previous sonar measurement (not filtered)
	hrt_abstime sonar_valid_time = 0;	// time of last sonar measurement used for correction (filtered)

	bool baro_init = false;
	bool acc_init = false;
	bool sonar_init = false;		// sonar time is initialized
	bool gps_valid = false;			// GPS is valid
	bool sonar_valid = false;		// sonar is valid
	bool sonar_vel_valid = false;	// sonar is valid
	bool flow_valid = false;		// flow is valid
	bool flow_accurate = false;		// flow should be accurate (this flag not updated if flow_valid == false)
	bool vision_valid = false;
	bool sonar_updated = false;

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator;
	memset(&actuator, 0, sizeof(actuator));
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct home_position_s home;
	memset(&home, 0, sizeof(home));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vision_position_estimate vision;
	memset(&vision, 0, sizeof(vision));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct range_finder_report sonar;
	memset(&sonar, 0, sizeof(sonar));

	/* subscribe */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int vision_position_estimate_sub = orb_subscribe(ORB_ID(vision_position_estimate));
	int home_position_sub = orb_subscribe(ORB_ID(home_position));
	int sensor_sonar_sub_fd = orb_subscribe(ORB_ID(sensor_range_finder));

	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);
	orb_advert_t vehicle_global_position_pub = -1;

	struct position_estimator_inav_params params;
	struct position_estimator_inav_param_handles pos_inav_param_handles;
	/* initialize parameter handles */
	parameters_init(&pos_inav_param_handles);

	/* first parameters read at start up */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_update); /* read from param topic to clear updated flag */
	/* first parameters update */
	parameters_update(&pos_inav_param_handles, &params);

	struct pollfd fds_init[1] = {
		{ .fd = sensor_combined_sub, .events = POLLIN },
	};

	
	// temporary
//	bool already_printed_flag=false;
	float verbose_signal1=0;
	int verbose_counter1=0;
	float verbose_signal2=0;
	int verbose_counter2=0;
	float verbose_signal3=0;
//	int verbose_counter3=0;
	float verbose_signal4=0;
//	float verbose_signal5=0;
//	int verbose_counter5=0;
//	float verbose_signal6=0;
//	int verbose_counter6=0;
//	float verbose_signal7=0;
//	int verbose_counter7=0;
	float verbose_signal8=0;
//	int verbose_counter8=0;
	float verbose_signal9=0;
//	int verbose_counter9=0;
	bool verbose_signal10=0;
//	int verbose_counter10=0;
	bool verbose_signal11=0;
//	int verbose_counter11=0;
	float verbose_signal12=0.0f;
//	bool verbose_signal13=false;
//	float verbose_signal14=0.0f;
//	float verbose_signal15=0.0f;
//	float verbose_signal16=0.0f;
	float verbose_signal17=0.0f;

	bool new_verbose_signal1=false;
//	int new_verbose_signal2=0;
//	int new_verbose_signal3=0;
//	int new_verbose_signal4=0;
	float new_verbose_signal5=0.0f;
	float new_verbose_signal6=0.0f;
	float new_verbose_signal7=0.0f;
	float new_verbose_signal8=0.0f;
	float new_verbose_signal9=0.0f;
	float new_verbose_signal10=0.0f;


	/* wait for initial baro value */
	bool wait_baro = true;

	thread_running = true;

	while (wait_baro && !thread_should_exit) {
		int ret = poll(fds_init, 1, 1000);

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");

		} else if (ret > 0) {
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (wait_baro && sensor.baro_timestamp != baro_timestamp) {
					baro_timestamp = sensor.baro_timestamp;

					/* mean calculation over several measurements */
					if (baro_init_cnt < baro_init_num) {
						if (isfinite(sensor.baro_alt_meter)) {
							baro_offset += sensor.baro_alt_meter;
							baro_init_cnt++;
						}

					} else {
						wait_baro = false;
						baro_offset /= (float) baro_init_cnt;
						warnx("baro offs: %d", (int)baro_offset);
						mavlink_log_info(mavlink_fd, "[inav] baro offs: %d", (int)baro_offset);
						local_pos.z_valid = true;
						local_pos.v_z_valid = true;
					}
				}
			}
		}
	}
	// Initialize baro measurement
	baro_new = sensor.baro_alt_meter;


	/* main loop */
	struct pollfd fds[1] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
	};
	
	while (!thread_should_exit) {
		int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */

			/* vehicle attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++;

			bool updated;

			/* parameter update */
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				parameters_update(&pos_inav_param_handles, &params);
			}

			/* actuator */
			orb_check(actuator_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
			}

			/* armed */
			orb_check(armed_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}

			/* sensor combined */
			orb_check(sensor_combined_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.accelerometer_timestamp != accel_timestamp) {
					// temporary - acc_bias
					new_verbose_signal10 = acc_bias[2];
					if (att.R_valid) {
						/* correct accel bias */
						sensor.accelerometer_m_s2[0] -= acc_bias[0];
						sensor.accelerometer_m_s2[1] -= acc_bias[1];
						sensor.accelerometer_m_s2[2] -= acc_bias[2];

						// temporary
						verbose_signal9=sensor.accelerometer_m_s2[2];
						//verbose_counter9++;

						/* transform acceleration vector from body frame to NED frame */
						for (int i = 0; i < 3; i++) {
							acc[i] = 0.0f;

							for (int j = 0; j < 3; j++) {
								acc[i] += PX4_R(att.R, i, j) * sensor.accelerometer_m_s2[j];
							}
						}

						acc[2] += CONSTANTS_ONE_G;

					} else {
						memset(acc, 0, sizeof(acc));
					}
					
					if (acc_init) {
						acc[2] = last_filtered_acc + 0.062f * (acc[2] - last_filtered_acc);
						last_filtered_acc = acc[2];
						
					} else {
/*						acc_ctr++;
						if (acc_ctr>10) {
							last_filtered_acc=acc[2];
							acc_init = true;
						}
*/
						last_filtered_acc=acc[2];
						acc_init = true;
					}
					
					acc_bias[2] = acc_bias[2] + 0.0003f*acc[2]; // <=> bias + const * (acc[2] - 0)
					
					// temporary - filtered z acceleration
					new_verbose_signal9 = acc[2];


					accel_timestamp = sensor.accelerometer_timestamp;
					accel_updates++;
				}

				if (sensor.baro_timestamp != baro_timestamp) {
					// warn: might be a good idea to take the sonar component from the corr_baro. But I am not confident about it, so I won't do it now
					// not good, but possible: corr_baro = baro_offset - sensor.baro_alt_meter - z_est[0];

					// Moving average filter with window of size 20
					baro_buf[baro_ptr] = sensor.baro_alt_meter;
					if(baro_ptr >= BARO_FILT_WIND_SIZE-1){
						baro_ptr = 0;
						baro_init = true;
					}else{
						baro_ptr++;
					}
					if (baro_init) {
						baro_new = 0;
						for (int i=0; i<BARO_FILT_WIND_SIZE; i++) {
							baro_new += baro_buf[i]/BARO_FILT_WIND_SIZE;
						}
					}else{
					// use unfiltered baro measurement if not initialised yet
						baro_new = sensor.baro_alt_meter;
					}

					// TODO DECIDE which of the two to keep!
					corr_baro = baro_new - baro_offset - z_est[0]; // estou 50% seguro. estou a basear o sinal no que deve estar no inertial filter correction
					//corr_baro = baro_offset - baro_new + z_est[0]; // closest to original
					// OR
					//corr_baro = - (baro_offset - baro_new) - z_est[0]; // this seems to be the opposite of what I want. baro_offset - baro_new > 0 , z_est < 0
					baro_timestamp = sensor.baro_timestamp;
					baro_updates++;

					// temporary - baro offset
					verbose_signal8=baro_offset;
//					verbose_counter8++;
					new_verbose_signal5 = sensor.baro_alt_meter;
					new_verbose_signal6 = corr_baro;
					new_verbose_signal7 = z_est[0];
					new_verbose_signal8 = baro_new;
				}
			}
			
			
			/* sonar */
			orb_check(sensor_sonar_sub_fd, &updated);

			if (updated) {
				
				//temporary - new sonar measurement - if 1
				//verbose_signal13=true;


				sonar_updated = true;
				orb_copy(ORB_ID(sensor_range_finder), sensor_sonar_sub_fd, &sonar);

				/* calculate time from previous update */
//				float flow_dt = flow_prev > 0 ? (flow.flow_timestamp - flow_prev) * 1e-6f : 0.1f;
//				flow_prev = flow.flow_timestamp;

				// temporary - measured sonar distance
				verbose_signal12=sonar.distance;
//				verbose_signal14=PX4_R(att.R, 2, 2);
//				verbose_signal15=fabsf(sonar.distance - sonar_new);

				if ((sonar.distance > 0.2f) &&
					(sonar.distance < 7.6f) &&
					(PX4_R(att.R, 2, 2) > 0.7f)) { // TODO understand this, otherwise change it

					// temporary - if 2
//					new_verbose_signal4 = 1;

					if (sonar_init){
						sonar_time_prev = sonar_time;
						sonar_last_valid = sonar_new;
					}else{
						sonar_last_valid = sonar.distance;
						sonar_init=true;
					}
					
					
					sonar_time = t;
					sonar_new = sonar.distance;
					corr_sonar = sonar.distance + z_est[0];
					corr_sonar_filtered += (corr_sonar - corr_sonar_filtered) * params.sonar_filt;
 
					
					acceptable_change += (sonar_new - sonar_last_valid) * params.sonar_filt; // changed sonar_filt from .05 to .1
// sonar_last_valid maintains the same value (last valid measurement) if the current measurement is not considered to be valid

					// temporary - corr_sonar_filtered
//					verbose_signal16=corr_sonar_filtered;
					verbose_signal17=corr_sonar;

					if (fabsf(corr_sonar) > params.sonar_err) { // changed sonar_err from .3 to .2
						// temporary - if 3
//						new_verbose_signal2 = 1;

						/* correction of altitude is too large: spike or new ground level? */
//						if (fabsf(corr_sonar - corr_sonar_filtered) > params.sonar_err) {
						if (fabsf(sonar_new - ( sonar_last_valid + acceptable_change)) > params.sonar_err) { // the idea is that if it is a new altitude, then the (acceptable_change +  last_valid_measurement)  will eventually catch up with the measurement of the sonar, so it is not a spike.
							/* spike detected, ignore */

							//new_verbose_signal3 = 1;

							corr_sonar = 0.0f; // do not correct
							sonar_new = sonar_last_valid; // maintain last sonar measurement
							sonar_valid = false;
						} else {
							/* new ground level */
							
							// temporary - if 4
//							new_verbose_signal3 = 0;


							/*
							surface_offset -= corr_sonar;
							surface_offset_rate = 0.0f;
							corr_sonar = 0.0f;
							corr_sonar_filtered = 0.0f;
							*/
							acceptable_change = 0.0f;
							sonar_valid_time = t;
							if(sonar_valid){
								// sonar measurement was already valid on the previous iteration
								sonar_vel_valid = true; // sonar measurement is valid two times in a row
							}else{
								sonar_vel_valid = false;
							}
							sonar_valid = true;
							/*
							local_pos.surface_bottom_timestamp = t;
							mavlink_log_info(mavlink_fd, "[inav] new surface level: %d", (int)surface_offset);
							 */
							//float new_altitude = corr_sonar - z_est[0]; // warn: sign correct?
							//mavlink_log_info(mavlink_fd, "[inav] new altitude level: %d", (int)new_altitude);
						}

					} else {
						// temporary
//						new_verbose_signal2 = 0;
//						new_verbose_signal3 = -1;

						/* correction is ok, use it */
						sonar_valid_time = t;
						if(sonar_valid){
							// sonar measurement was already valid on the previous iteration
							sonar_vel_valid = true;
						}else{
							sonar_vel_valid = false;
						}
						sonar_valid = true;
						acceptable_change = 0.0f;
					}
				}
				
				// temporary
//				else{new_verbose_signal4 = 0;}
			}
			
			//temporary - new sonar measurement
			else{
				//verbose_signal13=false;
				//new_verbose_signal2 = -1;
				//new_verbose_signal3 = -1;
				//new_verbose_signal4 = -1;
			}
			new_verbose_signal1 = sonar_valid;


			/* optical flow */
			orb_check(optical_flow_sub, &updated);
			
			if(updated){
				// DONE (after adding the sensor) separated flow from sonar

				float flow_q = flow.quality / 255.0f;
//				float dist_bottom = - z_est[0] - surface_offset;
				float dist_bottom = - z_est[0];
							if (dist_bottom > 0.2f && flow_q > params.flow_q_min && (t < sonar_valid_time + sonar_valid_timeout) && PX4_R(att.R, 2, 2) > 0.7f) {
					/* distance to surface */
					float flow_dist = dist_bottom / PX4_R(att.R, 2, 2);
					/* check if flow if too large for accurate measurements */
					/* calculate estimated velocity in body frame */
					float body_v_est[2] = { 0.0f, 0.0f };

					for (int i = 0; i < 2; i++) {
						body_v_est[i] = PX4_R(att.R, 0, i) * x_est[1] + PX4_R(att.R, 1, i) * y_est[1] + PX4_R(att.R, 2, i) * z_est[1];
					}

					/* set this flag if flow should be accurate according to current velocity and attitude rate estimate */
					flow_accurate = fabsf(body_v_est[1] / flow_dist - att.rollspeed) < max_flow &&
							fabsf(body_v_est[0] / flow_dist + att.pitchspeed) < max_flow;

					/* convert raw flow to angular flow (rad/s) */
					float flow_ang[2];
					//todo check direction of x und y axis
					flow_ang[0] = flow.pixel_flow_x_integral/(float)flow.integration_timespan*1000000.0f;//flow.flow_raw_x * params.flow_k / 1000.0f / flow_dt;
					flow_ang[1] = flow.pixel_flow_y_integral/(float)flow.integration_timespan*1000000.0f;//flow.flow_raw_y * params.flow_k / 1000.0f / flow_dt;
					/* flow measurements vector */
					float flow_m[3];
					flow_m[0] = -flow_ang[0] * flow_dist;
					flow_m[1] = -flow_ang[1] * flow_dist;
					flow_m[2] = z_est[1];
					/* velocity in NED */
					float flow_v[2] = { 0.0f, 0.0f };

					/* project measurements vector to NED basis, skip Z component */
					for (int i = 0; i < 2; i++) {
						for (int j = 0; j < 3; j++) {
							flow_v[i] += PX4_R(att.R, i, j) * flow_m[j];
						}
					}

					/* velocity correction */
					corr_flow[0] = flow_v[0] - x_est[1];
					corr_flow[1] = flow_v[1] - y_est[1];
					/* adjust correction weight */
					float flow_q_weight = (flow_q - params.flow_q_min) / (1.0f - params.flow_q_min);
					w_flow = PX4_R(att.R, 2, 2) * flow_q_weight / fmaxf(1.0f, flow_dist);

					/* if flow is not accurate, reduce weight for it */
					// TODO make this more fuzzy
					if (!flow_accurate) {
						w_flow *= 0.05f;
					}

					/* under ideal conditions, on 1m distance assume EPH = 10cm */
					eph_flow = 0.1f / w_flow;

					flow_valid = true;

				} else {
					w_flow = 0.0f;
					flow_valid = false;
				}

				flow_updates++;
			}

			if (sonar_updated && sonar_valid) {
				/* update baro offset */
				// When there is a new valid sonar measurement, we correct baro's bias (bias is always drifting)
				baro_offset += ((baro_new - baro_offset) + sonar_new) * 0.5f; // update baro_offset in a way that the measurement of the baro is closer to the measurement of the sonar (true altitude value)

				z_est[0] = - sonar_new; // set ground truth. equivalent to line 653
				corr_sonar = 0.0f;

				// temporary
/*				printf("------------------------------->Ground truth: %5.3f\n", (double)z_est[0]);
				already_printed_flag=true;
*/
				// TODO only take the false out when I create a flag for valid velocity, which is true when I have 2 consecutive good measurements (2 consecutive sonar_valid)
				if (sonar_init && sonar_vel_valid) { // if (sonar_init)
					z_est[1] = (z_est[0] - sonar_last_valid) / (sonar_time - sonar_time_prev); // warn maybe I should use previous z_est instead of the previous sonar measurement
				}

				sonar_updated = false;
			}



			/* home position */
			orb_check(home_position_sub, &updated);

			if (updated) {
				printf("[inav] Something wrong 1.\n");
				orb_copy(ORB_ID(home_position), home_position_sub, &home);

				if (home.timestamp != home_timestamp) {
					home_timestamp = home.timestamp;

					double est_lat, est_lon;
					float est_alt;

					if (ref_inited) {
						/* calculate current estimated position in global frame */
						est_alt = local_pos.ref_alt - local_pos.z;
						map_projection_reproject(&ref, local_pos.x, local_pos.y, &est_lat, &est_lon);
					}

					/* update reference */
					map_projection_init(&ref, home.lat, home.lon);

					/* update baro offset */
					baro_offset += home.alt - local_pos.ref_alt;

					local_pos.ref_lat = home.lat;
					local_pos.ref_lon = home.lon;
					local_pos.ref_alt = home.alt;
					local_pos.ref_timestamp = home.timestamp;

					if (ref_inited) {
						/* reproject position estimate with new reference */
						map_projection_project(&ref, est_lat, est_lon, &x_est[0], &y_est[0]);
						z_est[0] = -(est_alt - local_pos.ref_alt);
					}

					ref_inited = true;
				}
			}


			/* check no vision circuit breaker is set */
			if (params.no_vision != CBRK_NO_VISION_KEY) {
				/* vehicle vision position */
				orb_check(vision_position_estimate_sub, &updated);

				if (updated) {
					printf("[inav] Something wrong 2.\n");
					orb_copy(ORB_ID(vision_position_estimate), vision_position_estimate_sub, &vision);

					static float last_vision_x = 0.0f;
					static float last_vision_y = 0.0f;
					static float last_vision_z = 0.0f;

					/* reset position estimate on first vision update */
					if (!vision_valid) {
						x_est[0] = vision.x;
						x_est[1] = vision.vx;
						y_est[0] = vision.y;
						y_est[1] = vision.vy;
						/* only reset the z estimate if the z weight parameter is not zero */
						if (params.w_z_vision_p > MIN_VALID_W)
						{
							z_est[0] = vision.z;
							z_est[1] = vision.vz;
						}

						vision_valid = true;

						last_vision_x = vision.x;
						last_vision_y = vision.y;
						last_vision_z = vision.z;

						warnx("VISION estimate valid");
						mavlink_log_info(mavlink_fd, "[inav] VISION estimate valid");
					}

					/* calculate correction for position */
					corr_vision[0][0] = vision.x - x_est[0];
					corr_vision[1][0] = vision.y - y_est[0];
					corr_vision[2][0] = vision.z - z_est[0];

					static hrt_abstime last_vision_time = 0;

					float vision_dt = (vision.timestamp_boot - last_vision_time) / 1e6f;
					last_vision_time = vision.timestamp_boot;

					if (vision_dt > 0.000001f && vision_dt < 0.2f) {
						vision.vx = (vision.x - last_vision_x) / vision_dt;
						vision.vy = (vision.y - last_vision_y) / vision_dt;
						vision.vz = (vision.z - last_vision_z) / vision_dt;

						last_vision_x = vision.x;
						last_vision_y = vision.y;
						last_vision_z = vision.z;

						/* calculate correction for velocity */
						corr_vision[0][1] = vision.vx - x_est[1];
						corr_vision[1][1] = vision.vy - y_est[1];
						corr_vision[2][1] = vision.vz - z_est[1];
					} else {
						/* assume zero motion */
						corr_vision[0][1] = 0.0f - x_est[1];
						corr_vision[1][1] = 0.0f - y_est[1];
						corr_vision[2][1] = 0.0f - z_est[1];
					}

				}
			}


			// vehicle GPS position
			orb_check(vehicle_gps_position_sub, &updated);
/*
			if (updated) {
				printf("[inav] Something wrong 3.\n");
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				bool reset_est = false;

				// hysteresis for GPS quality
				if (gps_valid) {
					if (gps.eph > max_eph_epv || gps.epv > max_eph_epv || gps.fix_type < 3) {
						gps_valid = false;
						mavlink_log_info(mavlink_fd, "[inav] GPS signal lost");
					}

				} else {
					if (gps.eph < max_eph_epv * 0.7f && gps.epv < max_eph_epv * 0.7f && gps.fix_type >= 3) {
						gps_valid = true;
						reset_est = true;
						mavlink_log_info(mavlink_fd, "[inav] GPS signal found");
					}
				}

				if (gps_valid) {
					double lat = gps.lat * 1e-7;
					double lon = gps.lon * 1e-7;
					float alt = gps.alt * 1e-3;

					// initialize reference position if needed
					if (!ref_inited) {
						if (ref_init_start == 0) {
							ref_init_start = t;

						} else if (t > ref_init_start + ref_init_delay) {
							ref_inited = true;

							// set position estimate to (0, 0, 0), use GPS velocity for XY
							x_est[0] = 0.0f;
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = 0.0f;
							y_est[1] = gps.vel_e_m_s;

							local_pos.ref_lat = lat;
							local_pos.ref_lon = lon;
							local_pos.ref_alt = alt + z_est[0];
							local_pos.ref_timestamp = t;

							// initialize projection
							map_projection_init(&ref, lat, lon);
							// XXX replace this print
							warnx("init ref: lat=%.7f, lon=%.7f, alt=%8.4f", (double)lat, (double)lon, (double)alt);
							mavlink_log_info(mavlink_fd, "[inav] init ref: %.7f, %.7f, %8.4f", (double)lat, (double)lon, (double)alt);
						}
					}

					if (ref_inited) {
						// project GPS lat lon to plane
						float gps_proj[2];
						map_projection_project(&ref, lat, lon, &(gps_proj[0]), &(gps_proj[1]));

						// reset position estimate when GPS becomes good
						if (reset_est) {
							x_est[0] = gps_proj[0];
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = gps_proj[1];
							y_est[1] = gps.vel_e_m_s;
						}

						// calculate index of estimated values in buffer
						int est_i = buf_ptr - 1 - min(EST_BUF_SIZE - 1, max(0, (int)(params.delay_gps * 1000000.0f / PUB_INTERVAL)));
						if (est_i < 0) {
							est_i += EST_BUF_SIZE;
						}

						// calculate correction for position
						corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
						corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];
						corr_gps[2][0] = local_pos.ref_alt - alt - est_buf[est_i][2][0];

						// calculate correction for velocity
						if (gps.vel_ned_valid) {
							corr_gps[0][1] = gps.vel_n_m_s - est_buf[est_i][0][1];
							corr_gps[1][1] = gps.vel_e_m_s - est_buf[est_i][1][1];
							corr_gps[2][1] = gps.vel_d_m_s - est_buf[est_i][2][1];

						} else {
							corr_gps[0][1] = 0.0f;
							corr_gps[1][1] = 0.0f;
							corr_gps[2][1] = 0.0f;
						}

						// save rotation matrix at this moment
						memcpy(R_gps, R_buf[est_i], sizeof(R_gps));

						w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, gps.eph);
						w_gps_z = min_eph_epv / fmaxf(min_eph_epv, gps.epv);
					}

				} else {
					// no GPS lock
					memset(corr_gps, 0, sizeof(corr_gps));
					ref_init_start = 0;
				}

				gps_updates++;
			}
*/
		}

		/* check for timeout on FLOW topic */
		if (flow_valid && t > flow.timestamp + flow_topic_timeout) {
			flow_valid = false;
			warnx("FLOW timeout");
			mavlink_log_info(mavlink_fd, "[inav] FLOW timeout");
		}

		// warn: take out? There are 2 similar blocks
		/* check for timeout on sonar topic */
		if (sonar_valid && t > sonar_valid_time + sonar_valid_timeout) {
			sonar_valid = false;
			warnx("SONAR timeout");
			mavlink_log_info(mavlink_fd, "[inav] SONAR timeout");
		}

		/* check for timeout on GPS topic */
		if (gps_valid && (t > (gps.timestamp_position + gps_topic_timeout))) {
			gps_valid = false;
			warnx("GPS timeout");
			mavlink_log_info(mavlink_fd, "[inav] GPS timeout");
		}

		/* check for timeout on vision topic */
		if (vision_valid && (t > (vision.timestamp_boot + vision_topic_timeout))) {
			vision_valid = false;
			warnx("VISION timeout");
			mavlink_log_info(mavlink_fd, "[inav] VISION timeout");
		}

		// TODO uncomment this
		/* sonar measurements not valid for a short period */
/*		if (sonar_valid && (t > (sonar_time + sonar_timeout))) {
			corr_sonar = 0.0f;
			sonar_valid = false;
			warnx("SONAR timeout");
			mavlink_log_info(mavlink_fd, "[inav] SONAR timeout");
		}
*/

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.002);		// constrain dt from 2 to 20 ms
		t_prev = t;

		/* increase EPH/EPV on each step */
		if (eph < max_eph_epv) {
			eph *= 1.0f + dt;
		}
/*		if (epv < max_eph_epv) {
			epv += 0.005f * dt;	// add 1m to EPV each 200s (baro drift)
		}
*/
		/* use GPS if it's valid and reference position initialized */
		bool use_gps_xy = ref_inited && gps_valid && params.w_xy_gps_p > MIN_VALID_W;
		bool use_gps_z = ref_inited && gps_valid && params.w_z_gps_p > MIN_VALID_W;
		/* use VISION if it's valid and has a valid weight parameter */
		bool use_vision_xy = vision_valid && params.w_xy_vision_p > MIN_VALID_W;
//		bool use_vision_z = vision_valid && params.w_z_vision_p > MIN_VALID_W;
		/* use flow if it's valid and (accurate or no GPS available) */
		bool use_flow = flow_valid && (flow_accurate || !use_gps_xy);

		bool can_estimate_xy = (eph < max_eph_epv) || use_gps_xy || use_flow || use_vision_xy;

		bool dist_bottom_valid = (t < sonar_valid_time + sonar_valid_timeout);

		/*
		if (dist_bottom_valid) {
			// surface distance prediction
			surface_offset += surface_offset_rate * dt;

			// surface distance correction
			if (sonar_valid) {
				surface_offset_rate -= corr_sonar * 0.5f * params.w_z_sonar * params.w_z_sonar * dt;
				surface_offset -= corr_sonar * params.w_z_sonar * dt;
			}
		}
		*/

		float w_xy_gps_p = params.w_xy_gps_p * w_gps_xy;
		float w_xy_gps_v = params.w_xy_gps_v * w_gps_xy;
//		float w_z_gps_p = params.w_z_gps_p * w_gps_z;
//		float w_z_gps_v = params.w_z_gps_v * w_gps_z;

		float w_xy_vision_p = params.w_xy_vision_p;
		float w_xy_vision_v = params.w_xy_vision_v;
//		float w_z_vision_p = params.w_z_vision_p;

		/* reduce GPS weight if optical flow is good */
		if (use_flow && flow_accurate) {
			printf("[inav] Something wrong 3 1.\n");
			w_xy_gps_p *= params.w_gps_flow;
			w_xy_gps_v *= params.w_gps_flow;
		}

		// DONE implement baro offset correction with sonar. Line 611. Now I don't need to compensate with offs_corr because I am giving the true value for baro_offset and to corr_baro
		/* baro offset correction */
/*		if (use_gps_z) {
			printf("[inav] Something wrong 4.\n");
			float offs_corr = corr_gps[2][0] * w_z_gps_p * dt;
			baro_offset += offs_corr;
			corr_baro += offs_corr;
		}
*/

		/* accelerometer bias correction for GPS (use buffered rotation matrix) */
//		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };

/*		if (use_gps_xy) {
			printf("[inav] Something wrong 4 1.\n");
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}
		
		if (use_gps_z) {
			printf("[inav] Something wrong 4 2.\n");
			accel_bias_corr[2] -= corr_gps[2][0] * w_z_gps_p * w_z_gps_p;
			accel_bias_corr[2] -= corr_gps[2][1] * w_z_gps_v;
		}

		// (gps) transform error vector from NED frame to body frame
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R_gps[j][i] * accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
*/
/*
		// accelerometer bias correction for VISION (use buffered rotation matrix)
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_vision_xy) {
			printf("[inav] Something wrong 5.\n");
			accel_bias_corr[0] -= corr_vision[0][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[0] -= corr_vision[0][1] * w_xy_vision_v;
			accel_bias_corr[1] -= corr_vision[1][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[1] -= corr_vision[1][1] * w_xy_vision_v;
		}

		if (use_vision_z) {
			printf("[inav] Something wrong 6.\n");
			accel_bias_corr[2] -= corr_vision[2][0] * w_z_vision_p * w_z_vision_p;
		}

		// (vision) transform error vector from NED frame to body frame
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(att.R, j, i) * accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
*/
/*
		// accelerometer bias correction for flow and baro (assume that there is no delay)
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_flow) {
			printf("[inav] Something wrong 7.\n");
			accel_bias_corr[0] -= corr_flow[0] * params.w_xy_flow;
			accel_bias_corr[1] -= corr_flow[1] * params.w_xy_flow;
		}
*/
		// Previous approach to acceleration bias correction
/*		if (sonar_valid) {
			// if sonar is valid twice in a row
			//accel_bias_corr[2] -= corr_sonar * 0.14f * 0.14f; // This acc_bias implementation does not work
		}
		//accel_bias_corr[2] -= corr_baro * params.w_z_baro * params.w_z_baro;
*/

		// temporary - acceleration correction after baro corr , baro correction on acceleration
//		verbose_signal7=accel_bias_corr[2];
//		verbose_counter7++;
//		verbose_signal6=corr_baro * params.w_z_baro * params.w_z_baro * params.w_z_baro;
//		verbose_counter6++;


/*
		// transform error vector from NED frame to body frame
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += PX4_R(att.R, j, i) * accel_bias_corr[j];
			}

			if (isfinite(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
*/

		// temporary - total acc bias , z before prediction
//		verbose_signal5=acc_bias[2];
//		verbose_counter5++;
		verbose_signal1=z_est[0];
		verbose_counter1++;

		/* inertial filter prediction for altitude, based on last iteration's velocity estimation and corrected acceleration */
		
		inertial_filter_predict(dt, z_est, acc[2]);

		// temporary - z after prediction
		verbose_signal2=z_est[0];
		verbose_counter2++;

		if (!(isfinite(z_est[0]) && isfinite(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev, acc, corr_gps, w_xy_gps_p, w_xy_gps_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));
		}

		/* inertial filter correction for altitude */
		inertial_filter_correct(corr_baro, dt, z_est, 0, params.w_z_baro);

		// temporary - z after correction
		verbose_signal3=z_est[0];
//		verbose_counter3++;

		// TODO tune weight. maybe add velocity correction??
		// TODO check if corr_sonar is always zero inside the if. Should it be?
		if(sonar_valid){
			if (corr_sonar>0.0001f || corr_sonar<-0.0001f) {
				printf("[inav] Something wrong 7 1. CHECK bla bla nsh bla.\n");
			}
			// if z_est is corrected directly with sonar, then corr_sonar will be 0 here
			inertial_filter_correct(corr_sonar, dt, z_est, 0, sonar_weight);
		 }

/*		if (use_gps_z) {
			printf("[inav] Something wrong 7 2.\n");
			epv = fminf(epv, gps.epv);

			inertial_filter_correct(corr_gps[2][0], dt, z_est, 0, w_z_gps_p);
			inertial_filter_correct(corr_gps[2][1], dt, z_est, 1, w_z_gps_v);
		}
*/
/*		if (use_vision_z) {
			printf("[inav] Something wrong 8.\n");
			epv = fminf(epv, epv_vision);
			inertial_filter_correct(corr_vision[2][0], dt, z_est, 0, w_z_vision_p);
		}
*/
		if (!(isfinite(z_est[0]) && isfinite(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev, acc, corr_gps, w_xy_gps_p, w_xy_gps_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));
			memset(corr_gps, 0, sizeof(corr_gps));
			memset(corr_vision, 0, sizeof(corr_vision));
			corr_baro = 0;

		} else {
			memcpy(z_est_prev, z_est, sizeof(z_est));
		}

		if (can_estimate_xy) {
			printf("[inav] Something wrong 9.\n");
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);

			if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
				write_debug_log("BAD ESTIMATE AFTER PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev, acc, corr_gps, w_xy_gps_p, w_xy_gps_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
			}

			/* inertial filter correction for position */
			if (use_flow) {
				printf("[inav] Something wrong 10.\n");
				eph = fminf(eph, eph_flow);

				inertial_filter_correct(corr_flow[0], dt, x_est, 1, params.w_xy_flow * w_flow);
				inertial_filter_correct(corr_flow[1], dt, y_est, 1, params.w_xy_flow * w_flow);
			}

/*			if (use_gps_xy) {
				printf("[inav] Something wrong 11.\n");
				eph = fminf(eph, gps.eph);

				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp_velocity + gps_topic_timeout) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}
*/
			if (use_vision_xy) {
				printf("[inav] Something wrong 12.\n");
				eph = fminf(eph, eph_vision);

				inertial_filter_correct(corr_vision[0][0], dt, x_est, 0, w_xy_vision_p);
				inertial_filter_correct(corr_vision[1][0], dt, y_est, 0, w_xy_vision_p);

				if (w_xy_vision_v > MIN_VALID_W) {
					inertial_filter_correct(corr_vision[0][1], dt, x_est, 1, w_xy_vision_v);
					inertial_filter_correct(corr_vision[1][1], dt, y_est, 1, w_xy_vision_v);
				}
			}

			if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
				write_debug_log("BAD ESTIMATE AFTER CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev, acc, corr_gps, w_xy_gps_p, w_xy_gps_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
				memset(corr_gps, 0, sizeof(corr_gps));
				memset(corr_vision, 0, sizeof(corr_vision));
				memset(corr_flow, 0, sizeof(corr_flow));

			} else {
				memcpy(x_est_prev, x_est, sizeof(x_est));
				memcpy(y_est_prev, y_est, sizeof(y_est));
			}
		} else {
			/* gradually reset xy velocity estimates */
			inertial_filter_correct(-x_est[1], dt, x_est, 1, params.w_xy_res_v);
			inertial_filter_correct(-y_est[1], dt, y_est, 1, params.w_xy_res_v);
		}

		if (verbose_mode) {
			/* print updates rate */
			if (t > updates_counter_start + updates_counter_len) {
				float updates_dt = (t - updates_counter_start) * 0.000001f;
				warnx(
					"updates rate: accelerometer = %.1f/s, baro = %.1f/s, gps = %.1f/s, attitude = %.1f/s, flow = %.1f/s",
					(double)(accel_updates / updates_dt),
					(double)(baro_updates / updates_dt),
					(double)(gps_updates / updates_dt),
					(double)(attitude_updates / updates_dt),
					(double)(flow_updates / updates_dt));
				updates_counter_start = t;
				accel_updates = 0;
				baro_updates = 0;
				gps_updates = 0;
				attitude_updates = 0;
				flow_updates = 0;
			}
		}






		// temporary - prints
		printf("%.4f , %.4f , %.4f , %.4f , 0 , 0 , 0 , %4.4f , %.4f , %b , %b , ",
			   (double)verbose_signal1,
			   (double)verbose_signal2,
			   (double)verbose_signal3,
			   (double)verbose_signal4,
			   //(double)verbose_signal5,
			   //(double)verbose_signal6,
			   //(double)verbose_signal7,
			   (double)verbose_signal8,
			   (double)verbose_signal9,
			   verbose_signal10,
			   verbose_signal11);
		printf("0 , 0 , 0 , 0 , 0 , %.3f , 0 , 0 , 0 , 0 , %.4f , ",
/*			   verbose_counter1, // same as 2,3,4,5,6,7
			   verbose_counter8,
			   verbose_counter9,
			   verbose_counter10,
			   verbose_counter11,
*/			   (double)verbose_signal12,
			   //verbose_signal13,
//			   (double)verbose_signal14,
//			   (double)verbose_signal15,
//			   (double)verbose_signal16,
			   (double)verbose_signal17);
		printf("%b , 0 , 0 , 0 , ",
			   new_verbose_signal1);
//			   new_verbose_signal2,
//			   new_verbose_signal3,
//			   new_verbose_signal4,

		printf("%.4f , %.4f , %.4f , %.4f , %.4f , %.4f \n",
			   (double)new_verbose_signal5,
			   (double)new_verbose_signal6,
			   (double)new_verbose_signal7,
			   (double)new_verbose_signal8,
			   (double)new_verbose_signal9,
			   (double)new_verbose_signal10);



		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* push current estimate to buffer */
			est_buf[buf_ptr][0][0] = x_est[0];
			est_buf[buf_ptr][0][1] = x_est[1];
			est_buf[buf_ptr][1][0] = y_est[0];
			est_buf[buf_ptr][1][1] = y_est[1];
			est_buf[buf_ptr][2][0] = z_est[0];
			est_buf[buf_ptr][2][1] = z_est[1];

			/* push current rotation matrix to buffer */
			memcpy(R_buf[buf_ptr], att.R, sizeof(att.R));

			buf_ptr++;
			if (buf_ptr >= EST_BUF_SIZE) {
				buf_ptr = 0;
			}

			// temporary - final z estimate
			verbose_signal4=z_est[0];
			verbose_signal10=sonar_valid;
			//verbose_counter10++;
			verbose_signal11=sonar_updated;
			//verbose_counter11++;

			/* publish local position */
			local_pos.xy_valid = can_estimate_xy;
			local_pos.v_xy_valid = can_estimate_xy;
			local_pos.xy_global = local_pos.xy_valid && use_gps_xy;
			local_pos.z_global = local_pos.z_valid && use_gps_z;
			local_pos.x = x_est[0];
			local_pos.vx = x_est[1];
			local_pos.y = y_est[0];
			local_pos.vy = y_est[1];
			local_pos.z = z_est[0];
			local_pos.vz = z_est[1];
			local_pos.yaw = att.yaw;
			local_pos.dist_bottom_valid = dist_bottom_valid;
			local_pos.eph = eph;
//			local_pos.epv = epv;

			if (local_pos.dist_bottom_valid) {
//				local_pos.dist_bottom = -z_est[0] - surface_offset;
//				local_pos.dist_bottom_rate = -z_est[1] - surface_offset_rate;
				local_pos.dist_bottom = -z_est[0];
				local_pos.dist_bottom_rate = -z_est[1];
			}

			local_pos.timestamp = t;

			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

			if (local_pos.xy_global && local_pos.z_global) {
				/* publish global position */
				global_pos.timestamp = t;
				global_pos.time_utc_usec = gps.time_utc_usec;

				double est_lat, est_lon;
				map_projection_reproject(&ref, local_pos.x, local_pos.y, &est_lat, &est_lon);

				global_pos.lat = est_lat;
				global_pos.lon = est_lon;
				global_pos.alt = local_pos.ref_alt - local_pos.z;

				global_pos.vel_n = local_pos.vx;
				global_pos.vel_e = local_pos.vy;
				global_pos.vel_d = local_pos.vz;

				global_pos.yaw = local_pos.yaw;

				global_pos.eph = eph;
//				global_pos.epv = epv;

				if (vehicle_global_position_pub < 0) {
					vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

				} else {
					orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
				}
			}
		}
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[inav] stopped");
	thread_running = false;
	return 0;
}
// TODO change all that is GPS to come from the sonar. (almost) done
// TODO make sure that giving z_est[1] (velocity) from sonar measurements works
// TODO make acceptable_change dependent on z_est[0] (not sure. initially, it was. now I will have to think about it later)

// after getting to all the code errors:
// TODO tune params.sonar_err and params.sonar_filt to be able to get through disturbances without any damage to the estimate.

// TODO optimize filters


// don't forget later:
// TODO take out the prints "Something wrong"
// TODO remove dist_bottom if not needed
// TODO remove buffer if not needed
// TODO remove already_printed_flag
