/*
 * sens_packet.h
 *
 *  Created on: May 3, 2023
 *      Author: 18475
 */

#ifndef INC_SENS_PACKET_H_
#define INC_SENS_PACKET_H_

typedef struct packet_struct {
	uint8_t dev_id; //dev_id =0b1000101
	uint8_t stale_bits; // 0bXX[imu_acc][imu_gyr][mag][airspeed][bar][gps]
	uint32_t timestamp;
	float imu_acc_x;
	float imu_acc_y;
	float imu_acc_z;
	float imu_gyr_x;
	float imu_gyr_y;
	float imu_gyr_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float airspeed;
	float differential_pressure;
	float bar_altitude;
	float bar_press;
	float bar_temp; //ASK PHIL IF AVERAGING TEMPS MATTERS
	float gps_lat;
	float gps_long;
} sens_pkt_flt;

#endif /* INC_SENS_PACKET_H_ */
