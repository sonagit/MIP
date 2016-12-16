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
int controller(); // inner loop function
int disarm_controller();
int arm_controller();
int wait_for_starting_condition();
int zero_out_controller();

// threads
void* print_data(void* ptr);
void* setpoint_manager(void* ptr);
 
// Global variables
imu_data_t data; //struct to hold new data from IMU
d_filter_t LP, HP; // Lowpass and Highpass filters structs

float g_y, g_z, theta_a, filtered_theta_a, filtered_theta_g, theta; //gravity,thetas
float theta_dot, theta_g=0; // initialize starting angle for euler's method
float PhiLeft=0, PhiRight=0, Phi=0, Phi1=0, theta_r=0, theta_r1=0;//outer loop
float d1u=0, d1u1=0, d1u2=0, theta_e=0, theta_e1=0, theta_e2=0; // inner loop
float mount_angle = 0.4; // set angle of BBB on MIP
float offset = 0; // offset of gyro around X axis
const float TIME_STEP = 1.0/(float)SAMPLE_RATE; // Calc dt from sample rate

/*******************************************************************************
* arm_state_t
*
* ARMED or DISARMED to indicate if the controller is running
*******************************************************************************/
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;
arm_state_t arm_state = DISARMED;

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
    set_led(RED,1);
	set_led(GREEN,0);
	set_state(UNINITIALIZED);
    enable_motors();
    
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
	printf("   Accel XYZ(m/s^2)  |");
	printf("  theta_g (rad) |");
	printf("  theta_a (rad) |");
	printf("  theta  (rad)  |");
	printf("\n");
	
	// start print_data thread
    pthread_t print_data_thread;
    pthread_create(&print_data_thread, NULL, print_data, (void*) NULL);
    
	// start setpoint thread
	pthread_t  setpoint_thread;
	pthread_create(&setpoint_thread, NULL, setpoint_manager, (void*) NULL);

	// The interrupt function will print data when invoked
	set_imu_interrupt_func(&controller);
	
	set_state(RUNNING);
	
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING) {
		usleep(10000); // sleep for 10 ms
	}
	
	// exit cleanly
	disable_motors();
	power_off_imu();
	cleanup_cape();
	return 0;
}

/******************************************************************************
* int controller()
*
* balance MIP
*
******************************************************************************/
int controller(){
    
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
        disarm_controller();
	}
	
    // collect encoder positions, right wheel is reversed
	PhiRight = -1*(float)get_encoder_pos(3) * TWO_PI/(GEARBOX*60.0);
	PhiLeft =     (float)get_encoder_pos(2) * TWO_PI/(GEARBOX*60.0);
	    
    // Get average Phi
    Phi = (PhiLeft + PhiRight)/2.0 + theta;

    // Get desired theta from outer loop D2 controller
    theta_r = 1.6666*(Phi - 0.9975*Phi1) + 0.9608*theta_r1
    Phi1=Phi;
    theta_r1=theta;

    // theta error is reference theta - current theta 
    theta_e = theta_r - theta;

    // Control motors based on D1 controller
    d1u = 1.8372*d1u1 - 0.83725*d1u2 - 3.8333*theta_e + 7.3476*theta_e1 - 3.5171*theta_e2;

    // shift values down (I know I need to learn how to use the damn ring buffer)
	d1u2=d1u1;
	d1u1=d1u;
	theta_e2=theta_e1;
	theta_e1=theta_e;
    
	// exit if the controller is disarmed or state is exiting
	if(get_state() == EXITING){
		disable_motors();
		return 0;
	}
	if(arm_state==DISARMED){
		return 0;
	}
	printf("\r ");

    set_motor(2, d1u); // Left
    set_motor(3, -1*d1u); // Right (neg)
    printf("d1u: %6.2f",d1u);
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
        printf("\r");

		printf("%6.2f %6.2f %6.2f |",	data.accel[0],\
										    data.accel[1],\
										    data.accel[2]);
		printf("     %6.2f     |", filtered_theta_g); // Print angle from acc
		printf("     %6.2f     |", filtered_theta_a); // Print angle from gyro
		printf("     %6.2f     |", theta); // Print sum
		printf(" Phi: %6.2f",Phi);
		
		fflush(stdout); // flush to console (?)
		usleep(1000000);
	}
	return NULL;
}

/*******************************************************************************
* disarm_controller()
*
* disable motors & set the arm state to DISARMED
*******************************************************************************/
int disarm_controller(){
	disable_motors();
	arm_state = DISARMED;
	return 0;
}

/*******************************************************************************
* arm_controller()
*
* zero out the controller & encoders. Enable motors & arm the controller.
*******************************************************************************/
int arm_controller(){
	zero_out_controller();
	set_encoder_pos(ENCODER_CHANNEL_L,0);
	set_encoder_pos(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,theta);
	arm_state = ARMED;
	enable_motors();
	return 0;
}

/*******************************************************************************
* 	zero_out_controller()
*
*	Clear the controller's memory and zero out setpoints.
*******************************************************************************/
int zero_out_controller(){
	d1u = 0;
	d1u1 = 0;
	d1u2 = 0;
	theta_e = 0.0;
	theta_e1 = 0;
	theta_e2 = 0;
	Phi   = 0.0;
	Phi1 = 0;
	set_motor_all(0);
	return 0;
}

/*******************************************************************************
* int wait_for_starting_condition()
*
* Wait for MiP to be held upright long enough to begin.
* Returns 0 if successful. Returns -1 if the wait process was interrupted by
* pause button or shutdown signal.
*******************************************************************************/
int wait_for_starting_condition(){
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz;

	// exit if state becomes paused or exiting
	while(get_state()==RUNNING){
		// if within range, start counting
		if(fabs(theta) < START_ANGLE){
			checks++;
			// waited long enough, return
			if(checks >= checks_needed) return 0;
		}
		// fell out of range, restart counter
		else checks = 0;
		usleep(wait_us);
	}
	return -1;
}

/*******************************************************************************
* void* setpoint_manager(void* ptr)
*
* Detects pickup to control arming the controller.
*
*******************************************************************************/
void* setpoint_manager(void* ptr){

	// wait for IMU to settle
	disarm_controller();
	usleep(1000000);
	usleep(1000000);
	usleep(500000);
	set_state(RUNNING);
	set_led(RED,0);
	set_led(GREEN,1);
	
	while(get_state()!=EXITING){
		// sleep at beginning of loop so we can use the 'continue' statement
		usleep(1000000/SETPOINT_MANAGER_HZ);
		
		// nothing to do if paused, go back to beginning of loop
		if(get_state() != RUNNING) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(arm_state == DISARMED){
			if(wait_for_starting_condition()==0){
				zero_out_controller();
				arm_controller();
			}
			else continue;
		}
	}

	// if state becomes EXITING the above loop exists and we disarm here
	disarm_controller();
	return NULL;
}