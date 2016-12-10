/*******************************************************************************
* stubalance.c
* By: Stuart Sonatina
*
* Balance that there MIP
*
*******************************************************************************/

#include "../../libraries/roboticscape-usefulincludes.h"
#include "../../libraries/roboticscape.h"

#include "./stubalance_config.h"

#define SAMPLE_RATE 200 // Hz
#define TIME_CONSTANT 2.0 // Sec


// function declarations
int initialize_imu_dmp(imu_data_t *data, imu_config_t imu_config);
int set_imu_interrupt_func(int (*func)(void));
int inner_loop(); // inner loop function

// threads
void* print_data(void* ptr);

// variable declarations
imu_data_t data; //struct to hold new data from IMU
float g_y, g_z, theta_a, filtered_theta_a, filtered_theta_g, theta; // gravity, thetas
float theta_dot, theta_g = 0; // initialize starting angle for euler's method
float PhiLeft = 0, PhiRight = 0, PhiAvg;// init starting phi angles
float d1u=0, d1u1=0, d1u2=0, theta1=0, theta2=0;
float mount_angle = 0.39; // set angle of BBB on MIP
float offset = 0; // offset of gyro around X axis
const float TIME_STEP = 1.0/(float)SAMPLE_RATE; // Calc dt from sample rate
d_filter_t LP, HP; // Lowpass and Highpass filters structs

/******************************************************************************
* int main()
******************************************************************************/
int main(){

	// print welcome
	printf("\n-------------------------------------");
	printf("\n| Wow get ready for balance action! |\n");
	printf("-------------------------------------\n");
	
	// Initialize cape library
	if(initialize_cape()){
		printf("ERROR: failed to initialize_cape\n");
		return -1;
	}
    
    enable_motors();
    
    // start print_data thread
    pthread_t print_data_thread;
    pthread_create(&print_data_thread, NULL, print_data, (void*) NULL);
    
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
	printf("  theta  (rad)  |");
	printf("\n");
	
	// The interrupt function will print data when invoked
	set_imu_interrupt_func(&inner_loop);
	
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING) {
		usleep(100000); // sleep for 0.1 second
	}
	
	// exit cleanly
	//fclose(fp);
	disable_motors();
	power_off_imu();
	cleanup_cape();
	return 0;
}

/******************************************************************************
* void* inner_loop()
*
* Move motor in opposite direction of theta
*
******************************************************************************/
int inner_loop(){
    printf("\r ");

	// Integrate gyro data to get absolute position of theta
	theta_dot = (data.gyro[0] - offset)*DEG_TO_RAD; // spin rate in rad
	theta_g = theta_g + TIME_STEP*theta_dot; // euler's method
	// filter low freq noise out of gyro data
	filtered_theta_g = march_filter(&HP,theta_g + mount_angle);

    // calc theta from accelerometer G and Z components
	g_y = data.accel[1]-0.1;  // Y direction is 0.1 too high
	g_z = data.accel[2]-0.45; // Z direction is 0.45 too high
	theta_a = atan2(-g_z/9.8,g_y/9.8) + mount_angle; // angle to gravity
	// filter high freq noise out of accelerometer data
	filtered_theta_a = march_filter(&LP,theta_a);
    
    // add them together
    theta = filtered_theta_a + filtered_theta_g;
    
    // disable motors if MIP tips over
    if(fabs(theta)>TIP_ANGLE){
        disable_motors();
    }

    // collect encoder positions, right wheel is reversed
	PhiRight = -1*(float)get_encoder_pos(2) * TWO_PI/(GEARBOX*60.0);
	PhiLeft =     (float)get_encoder_pos(3) * TWO_PI/(GEARBOX*60.0);
	    
    // Get average Phi
    PhiAvg = (PhiLeft + PhiRight)/2.0;

    // Control motors based on D1 controller
    d1u = 1.8385*d1u1 - 0.82858*d1u2 - 3.8347*theta + 7.3529*theta1 - 3.5211*theta2;
    // shift values down (I know I need to learn how to use the damn ring buffer)
	d1u1=d1u;
	d1u2=d1u1;
	theta1=theta;
	theta2=theta1;
    set_motor(2, -1*d1u); // Right (neg)
    set_motor(3,d1u); // Left

	return 0;
}

/******************************************************************************
* void* print_data()
*
* Print data to console
*
******************************************************************************/
void* print_data(void* ptr){
    while(get_state()!=EXITING){
    	printf("%6.2f %6.2f %6.2f   |",	data.accel[0],\
    									data.accel[1],\
    									data.accel[2]);
    	printf("        %6.2f      |", filtered_theta_g); // Print angle from acc
    	printf("        %6.2f      |", filtered_theta_a); // Print angle from gyro
    	printf("        %6.2f      |", theta); // Print sum
    	printf("    %6.2f << PHIAVG",PhiAvg);
    	
    	fflush(stdout); // flush to console (?)
    	usleep(1000000);
    }
	return NULL;
}