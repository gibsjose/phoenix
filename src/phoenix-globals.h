/**********************************************************
*   Phoenix FSW - Globals
*
*   25 Jan 2016
*
*   Joe Gibson 						(joseph.gibson@nasa.gov)
*	Jordi Vila Hernandez de Lorenzo	(jordi.vila.hdl@gmail.com)
*
*   License: MIT (http://gibsjose.mit-license.org)
*
*   https://github.com/gibsjose/phoenix
**********************************************************/

#ifndef PHOENIX_GLOBALS_H
#define PHOENIX_GLOBALS_H

#define F_CPU 8000000UL     //Clock Speed (Hz): Note that USART is garbled when using 1MHz instead of 8MHz
#include "phoenix-receiver.h"
//Simple Error Representation
#define OK      0
#define ERROR   1


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Gains for the ROLL PID, are the same as the PITCH PID
#define P_ROLL  1.4
#define I_ROLL 0.05
#define D_ROLL 15

#define P_YAW 4
#define I_YAW 0.02
#define D_YAW 0

#define UPPER_LIMIT 400
#define LOWER_LIMIT -400

volatile receiver_inputs_t * receiver;
volatile setpoints_t* setpoints;


#endif//PHOENIX_GLOBALS_H
