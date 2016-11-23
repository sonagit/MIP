/*******************************************************************************
* my_read_sensors.c
*
* Uses the DMP mode to print the accelerometer data to the console at 10hz
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>


// function declarations
int initialize_imu_dmp(imu_data_t *data, imu_config_t imu_config);
int set_imu_interrupt_func(int (*func)(void));
int stop_imu_interrupt_func();
int print_data(); //prints data to console
imu_data_t data; //struct to hold new data

// IMU interrupt function that prints to console
int print_data(){
	printf("\r");

	// Print accelerometer data
    if(read_accel_data(&data)<0){
		printf("read_accel_data failed\n");
	}
    printf("%6.2f %6.2f %6.2f   |",	data.accel[0],\
									data.accel[1],\
            						data.accel[2]);
	float g_y = data.accel[1]-0.1;  // Y direction is 0.1 too high
	float g_z = data.accel[2]-0.45; // Z direction is 0.45 too high
	float theta = atan2(g_z/9.8,g_y/9.8); // angle to gravity
	printf("	%6.2f",-theta);
	
	fflush(stdout); // flush
	return 0;
}

int main(){
  
	// Initialize cape library
	if(initialize_cape()){
	  printf("ERROR: failed to initialize_cape\n");
	  return -1;
	}

	// set imu configuration to defaults
	imu_config_t imu_config = get_default_imu_config();
	//int set_imu_config_to_defaults(imu_config_t *imu_config);

	imu_config.orientation = ORIENTATION_Y_UP;
	imu_config.dmp_sample_rate = 10;
	
	if(initialize_imu_dmp(&data, imu_config)){
		printf("initialize_imu_failed\n");
		printf("ERROR: IMU might be toast\n");
		blink_led(RED, 5, 10);
		return -1;
	}
	printf("\nReady for some accelerometer data?!\n");
	
	// Print header for accelerometer data.
	printf("    Accel XYZ(m/s^2)	|");
	printf("	Angle to Gravity (rad)");
	printf("\n");
	
	// sets the interrupt function to print data immediately after the header.
	set_imu_interrupt_func(&print_data);
	
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){		
		// Read gyro
		if(read_gyro_data(&data)<0){
			printf("read gyro data failed\n");
		}
		/*
		// Integrate gyro data to get absolute position
		float theta_dot = data.gyro[0]*DEG_TO_RAD; // spin rate
		float offset = -0.5; // offset
		
		// Print gyro data	
		printf("	%6.1f", theta_dot);
		*/	
		
		usleep(100000); // sleep for 0.1 second
	}
	
	// exit cleanly
	power_off_imu();
	cleanup_cape();
	return 0;
}