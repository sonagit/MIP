/*******************************************************************************
* complementary_filter.c
* By: Stuart Sonatina
*
* Calculate theta from accelerometer and gyro data after running them through
* Low pass and high pass filters, respectively.
*
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>

#define SAMPLE_RATE 100
#define TIME_CONSTANT 2.0

// function declarations
int initialize_imu_dmp(imu_data_t *data, imu_config_t imu_config);
int set_imu_interrupt_func(int (*func)(void));
int print_data(); // prints IMU data

// variable declarations
imu_data_t data; //struct to hold new data from IMU
float g_y, g_z, theta_a, filtered_theta_a, filtered_theta_g, sum; // gravity, thetas
float theta_dot, theta_g = 0; // initialize starting angle for euler's method
float offset = -0.5; // offset of gyro around X axis
const float TIME_STEP = 1.0/(float)SAMPLE_RATE; // Calc dt from sample rate
d_filter_t LP, HP; // Lowpass and Highpass filters structs
char filename[] = "HW6_Acc-Gyro-Sum"; // file name for csv
FILE *fp; // Makes a file pointer to stream thing I have no idea really

/******************************************************************************
* int main()
******************************************************************************/
int main(){

	// print welcome
	printf("\n------------------------------");
	printf("\n| Welcome to filter madness! |\n");
	printf("------------------------------\n");
	
	// Initialize cape library
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
    
	// open file to append thetas to csv
	fp=fopen(strcat(filename,".csv"),"a"); // opens file to append

    
    // get yourself some filters
	LP = create_first_order_lowpass(TIME_STEP, TIME_CONSTANT);
	HP = create_first_order_highpass(TIME_STEP, TIME_CONSTANT);
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
	
	// Print header to console
	printf("    Accel XYZ(m/s^2)	|");
	printf("    theta_g (rad)   |");
	printf("    theta_a (rad)   |");
	printf("      sum  (rad)    |");
	printf("\n");
	
	// The interrupt function will print data when invoked
	set_imu_interrupt_func(&print_data);
	
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING) {
		usleep(100000); // sleep for 0.1 second
	}
	
	// exit cleanly
	fclose(fp);
	power_off_imu();
	cleanup_cape();
	return 0;
}

/******************************************************************************
* int print_data()
*
* IMU interrupt function that prints IMU data to console and csv file
*
******************************************************************************/
int print_data(){
	printf("\r ");

	// Integrate gyro data to get absolute position of theta
	theta_dot = (data.gyro[0] - offset)*DEG_TO_RAD; // spin rate in rad
	theta_g = theta_g + TIME_STEP*theta_dot; // euler's method
	// filter low freq noise out of gyro data
	filtered_theta_g = march_filter(&HP,theta_g);

    // calc theta from accelerometer G and Z components
	g_y = data.accel[1]-0.1;  // Y direction is 0.1 too high
	g_z = data.accel[2]-0.45; // Z direction is 0.45 too high
	theta_a = atan2(-g_z/9.8,g_y/9.8); // angle to gravity
	// filter high freq noise out of accelerometer data
	filtered_theta_a = march_filter(&LP,theta_a);
    
    // add them togeter
    sum = filtered_theta_a + filtered_theta_g;
	// Print data to console
	printf("%6.2f %6.2f %6.2f   |",	data.accel[0],\
									data.accel[1],\
									data.accel[2]);
	printf("        %6.2f      |", filtered_theta_g); // Print angle from acc
	printf("        %6.2f      |", filtered_theta_a); // Print angle from gyro
	printf("        %6.2f      |", sum); // Print sum
	
    fprintf(fp,"%6.2f,%6.2f,%6.2f\n",filtered_theta_g,filtered_theta_a,sum); // print
	
    fflush(fp); // flush to file
	fflush(stdout); // flush to console (?)
	return 0;
}
