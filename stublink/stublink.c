/*******************************************************************************
* stublink.c
*
* This is a blinking program!
* 1.	Pause button toggles RUNNING/PAUSED
* 2.	Mode button changes "MODE"
* 3.	Green/red LEDs change in different modes
* 4.	Main while loop prints status to screen
* 5.	Ctrl-C exits main while loop
* 6.	Sleeps inside while loop
* 7.	Prints "Goodbye Cruel World" after exit before main() returns
*******************************************************************************/

#include <usefulincludes.h>
#include <roboticscape.h>


// function declarations
int on_pause_pressed();
int on_pause_released();
int on_mode_released();

// mode toggles between blink patterns
int mode;

// toggle integer
int togg=0;

/*******************************************************************************
* int main()
*
* init
* blink when running
* solid red when paused
*******************************************************************************/
int main(){
	// always initialize cape library first
	initialize_cape();

	// do your own initialization here
	printf("\nWelcome to Stublink!\n");
	set_pause_pressed_func(&on_pause_pressed);
	set_pause_released_func(&on_pause_released);
	set_mode_released_func(&on_mode_released);

	// done initializing so set state to RUNNING
	set_state(RUNNING);
	// Start at 1 hz
	mode=0;
	// print state
	printf("State: Forrest\n");
	printf("Blinking at 1 hz            ");
	// Keep looping until state changes to EXITING
	while(get_state()!=EXITING){
		// Blink while running
		if(get_state()==RUNNING){
			// toggle led
			if(togg==0){
				set_led(GREEN, ON);
				set_led(RED, OFF);
				togg=1;
			} // end if
			else{
				set_led(GREEN, OFF);
				set_led(RED, ON);
				togg=0;
			} // end if
// 6. Sleep
// 3. Green/Red LEDs change in different modes
			switch(mode) {
				case 0: // blink at 1 hz
					printf("\rBlinking at 1 hz            ");
					usleep(1000000); // sleep for 1 second
					break;
				case 1: // blink at 5 hz
					printf("\rBlinking at 5 hz            ");
					usleep(200000); // sleep for 0.2 second
					break;
				case 2: // blink at 10 hz
					printf("\rBlinking at 10 hz           ");
					usleep(100000); // sleep for 0.1 second
					break;
				default: // blink at 20 hz if mode is something weird
					usleep(100000); // sleep for 0.05 second
					printf("\rMode is out of whack        ");
			} // end switch
			fflush(stdout);
		} // end if
		
		// Stop blinking if paused
		else if(get_state()==PAUSED){
			set_led(GREEN, OFF);
			set_led(RED, ON);
			usleep(100000); // sleep for 0.1 second
		} // end else if
	} // end while

// 7.	Print "Goodbye Cruel World"
	printf("\nGoodbye Cruel World\n");

	// exit cleanly
	cleanup_cape();
	return 0;
}


/*******************************************************************************
* int on_pause_released()
*
* 1. Make the Pause button toggle between paused and running states.
* 4. Print status only when status changes
*******************************************************************************/
int on_pause_released(){
	// toggle between paused and running modes
	if(get_state()==RUNNING){
		set_state(PAUSED);
		printf("\nState: Paused        \n");
		fflush(stdout);
	}
	else if(get_state()==PAUSED){
		set_state(RUNNING);
		printf("\nState: Forrest       \n");
		fflush(stdout);
	}
	return 0;
}

/*******************************************************************************
* int on_mode_released()
*
* 1. Mode button changes mode
*
* blink at
*  1 hz
*  5 hz
* 10 hz
*******************************************************************************/
int on_mode_released(){
	// cycle through modes
	if(mode<2)mode++;
	else mode=0;
	return 0;
}
/*******************************************************************************
* int on_pause_pressed()
*
* If the user holds the pause button for 2 seconds, set state to exiting which
* triggers the rest of the program to exit cleanly.
*******************************************************************************/
int on_pause_pressed(){
	int i=0;
	const int samples = 100;	 // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		usleep(us_wait/samples);
		if(get_pause_button() == RELEASED) return 0;
	}
	printf("\nlong press detected, shutting down\n");
	set_state(EXITING);
	return 0;
}
