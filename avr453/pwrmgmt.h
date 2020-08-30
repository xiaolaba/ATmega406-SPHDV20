/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for pwrmgmt.c
 *
 * \par Application note:
 *      AVR453: Smart Battery Reference Design
 *
 * \par Documentation:
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com \n
 *      Original author: Rob G. Fries - Apt Inc.\n
 *
 * $Revision: 2687 $
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/pwrmgmt.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//Values for SMCR
#define SLEEP_NONE 0
#define SLEEP_IDLE 0
#define SLEEP_ADCNR 2
#define SLEEP_POWERDOWN 4
#define SLEEP_POWERSAVE 6
#define SLEEP_POWEROFF 8


void DoShutdown(unsigned char reason);
#define SHUTDOWN_REASON_UNDERVOLTAGE 1
#define SHUTDOWN_REASON_CHARGE_OVERCURRENT 2
#define SHUTDOWN_REASON_DISCHARGE_OVERCURRENT 3
#define SHUTDOWN_REASON_OVERVOLTAGE 4
#define SHUTDOWN_REASON_SHORTCIRCUIT 5
#define SHUTDOWN_REASON_OVERTEMPERATURE 6
#define SHUTDOWN_REASON_UNKNOWNSTATE 7

void ChangePowerMode(unsigned char newmode, unsigned char shutdown_reason);
#define POWERMODE_POWEROFF 0
#define POWERMODE_POWERSAVE 1
#define POWERMODE_IDLE 2
#define POWERMODE_ACTIVE 3


#ifdef MODULE_PWRMGMT

  unsigned char PowerMode = 0;

#else

  extern unsigned char PowerMode;

#endif

