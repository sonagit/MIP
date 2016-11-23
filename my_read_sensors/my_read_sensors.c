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
	
// variable declarations
imu_data_t data; //struct to hold new data from IMU
float theta_g = 0; // initialize starting angle for euler's method
float offset = -0.5; // offset of gyro around X axis
char filename = HW5; // file name for csv
char fp;
// IMU interrupt function that prints to console
int print_data(){
	printf("\r ");

	// Print accelerometer data
    printf("%6.2f %6.2f %6.2f   |",	data.accel[0],\
									data.accel[1],\
            						data.accel[2]);
	float g_y = data.accel[1]-0.1;  // Y direction is 0.1 too high
	float g_z = data.accel[2]-0.45; // Z direction is 0.45 too high
	float theta_a = atan2(g_z/9.8,g_y/9.8); // angle to gravity
	printf("        %6.2f      ",-theta_a);
	
	// Integrate gyro data to get absolute position
	float theta_dot = (data.gyro[0] - offset)*DEG_TO_RAD; // spin rate in rad
	theta_g = theta_g + 0.1*theta_dot; // euler's method with t = 0.1
	// Print angle from gyro data
	printf("|    %6.1f    ", theta_g);
	
	// print thetas to csv file
	fp=fopen(filename,"w+"); // open file
	fprintf(fp,"%6.2f,%6.2f\n",theta_g,theta_a); // print to file
	fclose(fp);
	
	fflush(stdout); // flush
	return 0;
}
/******************************************************************************
* int main()
******************************************************************************/
int main(){
  
	// Initialize cape library
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}

	// set imu configuration to defaults
	imu_config_t imu_config = get_default_imu_config();
	
	imu_config.orientation = ORIENTATION_Y_UP; // change orientation to Y up
	imu_config.dmp_sample_rate = 20; // change sample rate
	
	if(initialize_imu_dmp(&data, imu_config)){
		printf("initialize_imu_dmp() failed\n");
		printf("ERROR: IMU might be toast\n");
		blink_led(RED, 5, 10);
		return -1;
	}
	
	// create CSV to record movements
	printf("\n Creating %s.csv file\n",filename);
	FILE *fp; // pointer to stream
	filename=strcat(filename,".csv"); // concat filename to .csv
	fp=fopen(filename,"w+"); // open file
	fprintf(fp,"theta_g,theta_a\n"); // print header to file
	fclose(fp);
	printf("%s.csv file created\n",filename);
	
	// print welcome
	printf("\nReady for some accelerometer data?!\n\n");
	
	// Print header to console
	printf("    Accel XYZ(m/s^2)	|");
	printf("    theta_g (rad)   |");
	printf("    theta_a (rad)");
	printf("\n");
	
	// sets the interrupt function to print data immediately after the header.
	set_imu_interrupt_func(&print_data);
	
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING) {
		usleep(10000); // sleep for 0.01 second
	}
	
	// exit cleanly
	power_off_imu();
	cleanup_cape();
	return 0;
}