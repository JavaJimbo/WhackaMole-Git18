/*
 *	Delay functions fro 16F877 running at 16 Mhz
 *	written by Jim Sedgwick 5/9/2006
 *	See DELAY16MHZ.H for details
 *
 *	Make sure this code is compiled with full optimization!!!
 */
// #include "DELAY16.H"

// Millisecond delay routine for 16 Mhz PICs
// Accepts any unsigned integer value
// Will create a delay in milliseconds
// for that value.

void DelayMs (unsigned int i) {
int j;
unsigned char dummy;

	dummy = 255;
	for(j=0; j<i; j++){
		while(dummy) dummy--;				
	}
}




 
