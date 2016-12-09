
/************************************************************************
;                                                                       ;
;                              Lamtec                                 	;
;                        Dual Control Shade                          	;
;                          	Motor.c                           			;
;             Routines for setting and maintaining motor                ;
;             		velocity using PID control               			;
;			 Also includes taskMotorSequencer() which					;
;				sets H bridge flags on the PIC							;
;				to turn on motor and change direction					;
;				         as necessary.									;
;																		;
;***********************************************************************/
#include "Project.h"        
#include "DualFunctions.h"
#include <stdio.h>
#include <stdlib.h>

unsigned char PIDStart = TRUE; // Flag indicates move is just starting so PID control variables get initialized.
unsigned char thresholdCounter = 0; // For detecting when speed has ramped up.
short rampTimer = 0; // Minimum time for ramping up motor
short leftToGo = 0; // Distance from final destination
unsigned short tmrMove = 0; // Max time allowed to complete move.
short commandVelocity = 0; // `B

#define INTEGRATOR_SIZE 128
short errorIntegrator[INTEGRATOR_SIZE];

#define DERIVATIVE_SIZE 12
short errorDerivative[DERIVATIVE_SIZE];

void resetIntegrator(void) {
    unsigned short i;
    for (i = 0; i < INTEGRATOR_SIZE; i++) errorIntegrator[i] = 0;
    for (i = 0; i < DERIVATIVE_SIZE; i++) errorDerivative[i] = 0;
}

// StartShadeMoving()
// This routine is called to start a new move.
//
// It initializes global variables so that the
// PIDcontrol() routine can move shade 
// to the position indicated by the input Destination.
// Command Velocity is the desired shade speed.
//
// If the destination is already within DEADBAND counts of current position,
// move is canceled.
//
// If the move was initiated by a command sent by host,
// then the PID routine will control motor by tracking position.
// However if the user is pulling on chain to move shade manually
// along motor, then "velocityMode" flag is set
// so PID routine will monitor velocity instead of position.

unsigned char StartShadeMoving(unsigned char mode, short Destination, short TargetVelocity) {
    short deltaPos;
    short breakDistance;

    thresholdCounter = 0;
    velocityMode = mode;
    hostCommand = 0;

    commandVelocity = TargetVelocity; // `B

    runThreshold = commandVelocity / 2; // RUN threshold is half desired speed, when RAMP becomes RUN state.
    haltThreshold = commandVelocity / 4; // If speed gets reduced to 1/4, assume user is trying to halt with chain.

    tmrMove = MAX_MOVE_TIME; // Set maximum allowable time to complete move		    

#define MAX_BREAK_DISTANCE 4 // Adjust destination by a few counts to compensate for speed
#define SPAN_VELOCITY (MAX_VELOCITY - MIN_VELOCITY)

    breakDistance = ((commandVelocity - MIN_VELOCITY) * MAX_BREAK_DISTANCE) / SPAN_VELOCITY;    
    printf ("\rBREAK DISTANCE: %d", breakDistance);

    // Determine move direction from destination and set travel direction:
    if (Destination > posCurrent) {
        travelDir = UP;
        destinationPos = Destination - breakDistance;
        state_motor = S_MOTOR_FWD;
    } else {
        travelDir = DOWN;
        destinationPos = Destination + breakDistance;        
        state_motor = S_MOTOR_REV;
    }

    deltaPos = abs(destinationPos - posCurrent); // Check whether current position is already close to destination.
    if (deltaPos <= DEADBAND) { // If it is, then don't initiate move.
        PIDstate = HALT;
#ifdef DIAGNOSTICS		
        printf("\r < DEADBAND: HALT");
#endif		
        return (FALSE); // MOVE CANCELLED   
    }

    rampTimer = 0; // General purpose timer used for both ramping up and ramping down.    
    ResetUserAssist(MOVE_INHIBIT_TIME); // User chain pulls are ignored during MOVE_INHIBIT_TIME, 
    // while speed ramps up in MOVE STATE.
    // This prevents false USER HALTs on a slow or heavy shade as it picks up speed.
    printf("\rSTART MOVE TO: %d", Destination);

    PIDStart = TRUE; // This flag prompts the PIDcontrol() routine below to initialize variables at start of move.
    motorPWM = 0; // Set PWM to zero to start with	    
    PIDstate = RAMP; // Start PID state machine with speed ramp up.    

    return (TRUE); // Everything OK - shade ready to start moving
}



/*************************************************************************
 *                                                                        *
 *   task_motorpwm() Tracks progress of move, sets PID state.             *	
 *                                                                        *
 **************************************************************************/
// This routine sets the PID state according to where
// the shade is in the current move.
// 
// Speed control should be a trapezoid,
// so PID control has the following ramp states:
// RAMP, RUN, RAMP_DOWN, HALT, STANDBY	

void task_motorpwm(void) {


    // PID MOTOR CONTROL				
    // If PIDstate is nonzero, then shade is commanded to go somewhere,
    // so motor is on and Direction is set below.
    // Velocity is set by PID controller.
    // When destination is reached, motor gets turned off.

    if (PIDstate) {
        if (tmrMove) tmrMove--; // Check for MOVE timeout
        else {
            printf("\rMOVE TIMEOUT");
            PIDstate = HALT; // then quit
        }
        // IF SHADE IS MOVING UNDER PID CONTROL, DO THE FOLLOWING:
        if (PIDstate != HALT) {
            // Determine remaining distance to destination: set leftToGo variable
            // If overrun correction is enabled, include UP or DOWN overrun in calculation:
            if (travelDir == UP) {
                state_motor = S_MOTOR_FWD;
                leftToGo = destinationPos - posCurrent;
            } else {
                state_motor = S_MOTOR_REV;
                leftToGo = posCurrent - destinationPos;
            }
            if (leftToGo < 0) leftToGo = 0;

            // PID STATE MACHINE
            // Initial PID state is RAMP - when speed picks up, switch  to RUN state
            // In RUN state, speed is kept constant.
            // Actual speed control is done in PIDcontrol(), 
            // which is called below:
            if (PIDstate == RAMP) {
                rampTimer++; // Allow minimum time to ramp speed so taskUserAssist() can avoid false user halts.
                if (rampTimer > MIN_RAMP_TIME) {
                    if (velocityAbsolute > runThreshold) { // If velocity is fast enough for RUN state,
                        thresholdCounter++;
                        if (thresholdCounter > 4) { // make sure speed really has picked up,
                            PIDstate = RUN; // then switch to RUN, but first    
                            printf("\rRUN");
                            enableUserHalts(); // Now OK to monitor user chain pulling. 
                        }
                    } else thresholdCounter = 0;
                }
            }

            // *****************  PWM GETS UPDATED BY PID HERE ***************************						
            // NOW UPDATE PID CONTROL            
            if (flagBEEPING) // Make sure beeper is silent
                motorPWM = 0; // If it isn't, don't run PID, set PWM = 0
            else PIDcontrol(); // otherwise, use PID control to set motor PWM
        } // END if (PIDstate!=HALT)        
        
        if (PIDstate == HALT || leftToGo == 0) { // If a HALT has occurred,	            
            state_motor = S_MOTOR_OFF; // make sure motor is turned off
            motorPWM = 0;
            PIDstate = STANDBY; // and go to STANDBY mode.						            
        }
    }
}

//------------------------------------------------------------------
// MOTOR DIRECTION CONTROL - SEQUENCER  
//------------------------------------------------------------------

// This routine determines which of the two PWM outputs
// to use for setting the duty cycle to move in the desired 
// direction at the PWM value set by the global motorPWM.
//
// Prerequisites: 
// The global state_motor should be set 
// to indicate desired motor direction
// and motorPWM should be set to PWM value 0-800.
// Also Factory Cal must have been done,
// so MotorPolarity and shade orientation 
// will use correct PWMs for desired direction.

void taskMotorSequencer(void) {

    switch (state_motor) {
        case S_MOTOR_OFF:
            CCP4CON = CCP2CON = 0x00; // Disable PWM outputs so RB0 and RB3 become IO pins
            PORTBbits.RB0 = PORTBbits.RB3 = 0; // Set them low
            TRISB = TRISB_INIT; // Reinitialize Port B just in case - RB0 and RB3 are outputs
            RBPU = 0; // Make sure pullups on Port B input pins are enabled
            break;

        case S_MOTOR_FWD:
            CCP4CON = CCP2CON = 0x0C; // Enable PWM outputs on RB0 and RB3
            TRISB = TRISB_INIT; // Reinitialize Port B just in case - RB0 and RB3 are outputs
            RBPU = 0; // Make sure pullups on Port B input pins are enabled
            if ((orientation == COUNTER_CLOCKWISE && MotorPolarity == POLARITY_OK)
                    || (orientation != COUNTER_CLOCKWISE && MotorPolarity != POLARITY_OK)) {
                setDutyPWMOne((unsigned short) motorPWM);
                setDutyPWMTwo(0);
            } else {
                setDutyPWMOne(0);
                setDutyPWMTwo((unsigned short) motorPWM);
            }
            break;

        case S_MOTOR_REV:
            CCP4CON = CCP2CON = 0x0C; // Enable PWM outputs on RB0 and RB3
            TRISB = TRISB_INIT; // Reinitialize Port B just in case - RB0 and RB3 are outputs
            RBPU = 0; // Enable pullups on Port B input pins                                               
            if ((orientation == COUNTER_CLOCKWISE && MotorPolarity == POLARITY_OK)
                    || (orientation != COUNTER_CLOCKWISE && MotorPolarity != POLARITY_OK)) {
                setDutyPWMOne(0);
                setDutyPWMTwo((unsigned short) motorPWM);
            } else {
                setDutyPWMOne((unsigned short) motorPWM);
                setDutyPWMTwo(0);
            }
            break;

        default:
            setDutyPWMOne(0);
            setDutyPWMTwo(0);
            break;
    }
}



/*******************************************************************************
 *                                                                        
 *   PIDcontrol() - Routine for computing PIDerror and correction in shade position
 *   
 ********************************************************************************
 * 
 * PIDcontrol() 
 * This routine adjusts the PWM value during a move
 * to maintain speed or position. 
 * 
 * Speed control should be a trapezoid,
 * so PID control has the following ramp states:
 * RAMP, RUN, RAMP_DOWN, HALT, STANDBY	
 * PID state is in STANDBY when shade is doing nothing,
 * which is the case most of the time.
 * 
 * If user initiates move by pulling on chain, 
 * and keeps pulling shade along with motor,
 * then constant velocity control is used.   
 * Otherwise if move was commanded by remote host
 * controller, then PID position control is used.
 * 
 * When user is pulling chain we don't have to worry
 * about speed matching with other shades,
 * so precision PID control really isn't necessary.
 * 
 * The constants KP, KI, KD
 * do proportional, integral, and derivative
 * correction of the measured error. 
 * For normal mode, "PIDerror" is the difference between actual
 * and commanded shade position.
 * In velocity mode, PIDerror is the difference
 * between actual and desired speed.	
 * 
 * All calculations are done using integers instead of
 * floating point variables. So MULTIPLIER 
 * is used to avoid roundoff errors.
 */

#define	MULTIPLIER 		1000	
#define PIDCONVERTER	100000     
#define PWM_SLEW 2


unsigned char PIDcontrol(void) {
    static unsigned char saturation = FALSE; // Flag used to prevent PID integrator from "winding up" (overflowing)	
    long KPcorr = 0; // Proportional correction
    long velError; // Velocity error - used for velocity control mode
    long derError; //  derivativeError;
    long KDcorr; // Derivative correction used for normal PID control mode.    
    long KIcorr = 0; // Integral correction		    
    long KVcorr = 0;  // Constant velocity correction
    long PIDcorrection = 0; // Total correction including KP, KI, KD correction    
    long lngPosition = 0; // Current position scaled for math
    static long cmdPos; // Desired (commanded) position - is always positive	
    static long lngDestPosition; // Scaled target position
    static long sumErrors = 0; // Running sum of position error for integrating errors.
    static long PIDvelocity = 0;
    static short PWMoffset = 100; // Sets base PWM value 
    static short motorPWMfixed;
    static unsigned short integratorIndex = 0;
    static unsigned short derivativeIndex = 0;
    short intError;
    static short posStart = 0;
    static unsigned char previousDir = 0;
#ifdef DIAGNOSTICS    
    static unsigned char displayCounter = 0; // For diagnostic output 
    short KPint, KIint, KDint, KVint; // For diagnostic output        
#endif    


    if (state_motor == S_MOTOR_OFF) // If motor is off, exit without calculating anything
        return (0);    

    lngPosition = (long) posCurrent; // Get current position and scale it up to do math
    lngPosition = lngPosition*MULTIPLIER;

    // START MOVE - INITIALIZE VARIABLES:
    if (PIDStart) { // This initializes variables once per move on the first pass.
        resetIntegrator();
        integratorIndex = 0;
        derivativeIndex = 0;
        PIDStart = FALSE;
        cmdPos = lngPosition; // Commanded position at start is current position.
        sumErrors = 0; // Clear integrator
        saturation = FALSE; // Reset integrator saturation flag		
        lngDestPosition = (long) destinationPos; // Get the destination position
        lngDestPosition = lngDestPosition * MULTIPLIER; // and scale it up for PIDerror math	        
        posStart = posCurrent;
        if (velocityMode) { // For constant velocity mode, set offset to high PWM,
            if (travelDir == UP) PWMoffset = PWM_MAX;
            else PWMoffset = THREEQUARTER_PWM_MAX;            
            PIDvelocity = USER_ASSIST_VELOCITY; // and start commanded velocity right at high speed - don't ramp up.
        } else { // Otherwise for normal PID mode:
            PWMoffset = 100; // set PWM offset to minimum to overcome friction,            
            PIDvelocity = MIN_VELOCITY;
        }
        previousDir = travelDir;
    }

    if (!velocityMode && PIDvelocity != commandVelocity) {
        if (abs(posStart - posCurrent) > 32) PIDvelocity = commandVelocity;
    }

    // GET POSITION ERROR FOR SHADE MOVING UP			
    if (travelDir == UP) {
        cmdPos = cmdPos + PIDvelocity; // Increment commanded position at desired velocity
        if (cmdPos > lngDestPosition) cmdPos = lngDestPosition; // Make sure shade isn't overshooting destination		
        PIDerror = lngPosition - cmdPos; // Error will be negative for slow shade, positive for fast shade.	
    }// GET POSITION ERROR FOR SHADE MOVING DOWN
    else {
        cmdPos = cmdPos - PIDvelocity; // Decrement commanded position at desired velocity
        if (cmdPos < lngDestPosition) cmdPos = lngDestPosition; // Make sure shade isn't overshooting destination		
        PIDerror = cmdPos - lngPosition; // Error will be negative for slow shade, positive for fast shade.			
    }

    if (PIDerror > 32767) intError = 32767;
    else if (PIDerror < -32767) intError = -32767;
    else intError = (short) PIDerror;

    // GET INTEGRAL CORRECTION:
    if (saturation == FALSE) {
        sumErrors = sumErrors - errorIntegrator[integratorIndex];
        errorIntegrator[integratorIndex] = intError;
        sumErrors = sumErrors + errorIntegrator[integratorIndex];
        KIcorr = (long) KI * sumErrors;
        integratorIndex++;
        if (integratorIndex >= INTEGRATOR_SIZE) integratorIndex = 0;
    }

    // GET VELOCITY OR DERIVATIVE CORRECTION (depending on Mode)
    if (velocityMode) {
        velError = (long) (velocityAbsolute - USER_ASSIST_VELOCITY) * MULTIPLIER;
        KVcorr = (long) KV * velError;
    } else {
        derError = (long) intError - (long) errorDerivative[derivativeIndex];
        KDcorr = (long) KD * derError;
        errorDerivative[derivativeIndex] = intError;
        derivativeIndex++;
        if (derivativeIndex >= DERIVATIVE_SIZE) derivativeIndex = 0;
    }

    // GET PROPORTIONAL CORRECTION:    
    KPcorr = (long) KP * PIDerror;

    // GET TOTAL CORRECTION AND CONVERT TO PWM: 
    if (velocityMode) PIDcorrection = KVcorr / PIDCONVERTER;
    else PIDcorrection = (KPcorr + KIcorr + KDcorr) / PIDCONVERTER;

    // If User is pulling on chain in same direction as motor,
    // switch to constant velocity mode and ramp up PWM         
    if (!velocityMode && UserState == USER_ASSIST) {
        velocityMode = TRUE;
        PIDvelocity = USER_ASSIST_VELOCITY;
        haltThreshold = (short) (PIDvelocity / 4);
        if (travelDir == UP) PWMoffset = PWM_MAX;
        else PWMoffset = THREEQUARTER_PWM_MAX;
    } 
     
    // If shade is being homed, then we have to `B
    // avoid accelerating through the dead zone at bottom.    
    // So after motor ramps up, use fixed speed:
    if (state_sys == S_STATE_GOTO0 && PIDstate == RUN)
        motorPWM = motorPWMfixed;
        // NORMAL CASE: Motor PWM is set to fixed offset plus correction:
    else motorPWM = PWMoffset - PIDcorrection;

    // For GO TO ZERO: store ramp speed PWM 
    // to use for RUN (see above statement)
    if (PIDstate == RAMP) motorPWMfixed = motorPWM;

    saturation = FALSE;
    if (motorPWM >= PWM_MAX) {
        motorPWM = PWM_MAX;
        saturation = TRUE; // Don't allow integrator to work if it is increasing saturated PWM
    } else if (motorPWM < 0) motorPWM = 0;

    // IF DIAGNOSTICS IS ENABLED, SEND DATA TO UART:
#ifdef DIAGNOSTICS	   

    if (enableDisplayMode) {
        if (displayCounter) displayCounter--;
        if (!displayCounter) {
            displayCounter = 10;

            KPint = (short) (KPcorr / MULTIPLIER);
            KIint = (short) (KIcorr / MULTIPLIER);
            KDint = (short) (KDcorr / MULTIPLIER);
            KVint = (short) (KVcorr / MULTIPLIER);
            
            if (velocityMode) intError = velError / MULTIPLIER;
            else intError = intError / MULTIPLIER;

            if (velocityMode) printf("\r>>>VEL: %d, ERR: %d, KV: %d, PWM: %d", velocityAbsolute, intError, KVint, motorPWM);
            else printf("\rPOS: %d, ERR: %d, KP: %d, KI: %d, KD: %d, PWM: %d", posCurrent, intError, KPint, KIint, KDint, motorPWM);
        }

    }

#endif	

    return (1); // Return 1 indicating motor is on and under PID control
}
