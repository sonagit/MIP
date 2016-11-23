/*******************************************************************************
* my_read_sensors.c
*
* Uses the DMP mode to print the accelerometer data to the console at 10hz
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>


// function declarations
int initialize_imu_dmp(imu_data_t *data, imu_config_t conf);
int set_imu_interrupt_func(int (*func)(void));
int stop_imu_interrupt_func();

int main(){
  
  imu_data_t data; //struct to hold new data
	
	// Initialize cape library
	if(initialize_cape()){
	  printf("ERROR: failed to initialize_cape\n");
	  return -1;
	}

	// default imu configuration
	imu_config_t conf = get_default_imu_config();
	int set_imu_config_to_defaults(imu_config_t *conf);

	if(initialize_imu(&data, conf)){
		printf("initialize_imu_failed\n");
		return -1;
	}

	printf("\nReady for some accelerometer data?!\n");

	// Print header for accelerometer data.
  printf("   Accel XYZ(m/s^2)  \n");


	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){
    printf("\r");
    
    // Print accelerometer data
    if(read_accel_data(&data)<0){
			printf("read_accel_data failed\n");
		}
    printf("%6.2f %6.2f %6.2f ",	data.accel[0],\
            											data.accel[1],\
            											data.accel[2]);
		
		fflush(stdout); // flush
		usleep(100000); // sleep for 0.1 second
	}
	
	// exit cleanly
	power_off_imu();
	cleanup_cape();
	return 0;
}