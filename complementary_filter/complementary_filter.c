/*******************************************************************************
* complementary_filter.c
*
* This is a filter....
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>

#define SAMPLE_RATE 100
#define TIME_CONSTANT 1.0


// function declarations
int initialize_imu_dmp(imu_data_t *data, imu_config_t imu_config);
int set_imu_interrupt_func(int (*func)(void));
int stop_imu_interrupt_func();
int print_data(); //prints data to console

// variable declarations
imu_data_t data; //struct to hold new data from IMU
float g_y, g_z, theta_a, filtered_theta_a, filtered_theta_g; // gravity, thetas
float theta_dot, theta_g = 0; // initialize starting angle for euler's method
float offset = -0.5; // offset of gyro around X axis
char filename[] = "HW6_Filtered"; // file name for csv
d_filter_t LP, HP; // Lowpass and Highpass filters

// IMU interrupt function that prints to console
int print_data(){
	printf("\r ");

	// Print accelerometer data
	printf("%6.2f %6.2f %6.2f   |",	data.accel[0],\
									data.accel[1],\
                  data.accel[2]);
	g_y = data.accel[1]-0.1;  // Y direction is 0.1 too high
	g_z = data.accel[2]-0.45; // Z direction is 0.45 too high
	theta_a = atan2(-g_z/9.8,g_y/9.8); // angle to gravity
	
	// filter high freq noise out of accelerometer data
	filtered_theta_a = march_filter(&LP,theta_a);
	printf("        %6.2f      ", filtered_theta_a);

	// Integrate gyro data to get absolute position
	theta_dot = (data.gyro[0] - offset)*DEG_TO_RAD; // spin rate in rad
	theta_g = theta_g + 0.01*theta_dot; // euler's method with t = 0.01

	// filter low freq noise out of gyro data
	filtered_theta_g = march_filter(&HP,theta_g);
	// Print angle from gyro data
	printf("|    %6.1f    ", filtered_theta_g);

	// print thetas to csv file
	FILE *fp;
	fp=fopen(filename,"a"); // open file to append
	fprintf(fp,"%6.2f,%6.2f\n",filtered_theta_g,filtered_theta_a); // print
	fclose(fp); // close file

	fflush(stdout); // flush
	return 0;
}
/******************************************************************************
* int main()
******************************************************************************/
int main(){
  printf("\nWelcome to filter madness!\n");

	// Initialize cape library
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}

  // get yourself some filters
  LP = create_first_order_lowpass(1.0/SAMPLE_RATE, TIME_CONSTANT);
  HP = create_first_order_highpass(1.0/SAMPLE_RATE, TIME_CONSTANT);
  
  // reset them filters
  reset_filter(&LP);
  reset_filter(&HP);

	// set imu configuration to defaults
	imu_config_t imu_config = get_default_imu_config();

	imu_config.orientation = ORIENTATION_Y_UP; // change orientation to Y up
	imu_config.dmp_sample_rate = SAMPLE_RATE;  // change sample rate
	
	if(initialize_imu_dmp(&data, imu_config)){
		printf("initialize_imu_dmp() failed\n");
		printf("ERROR: IMU might be toast\n");
		blink_led(RED, 5, 10);
		return -1;
	}
	
	// create CSV to record movements
	printf("\nCreating %s.csv file\n",filename);
	FILE *fp; // pointer to stream
	fp=fopen(strcat(filename,".csv"),"w"); // create empty file to write
	fprintf(fp,"filtered_theta_g,filtered_theta_a\n"); // print header to file
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
	fclose(fp);
	power_off_imu();
	cleanup_cape();
	return 0;
}
