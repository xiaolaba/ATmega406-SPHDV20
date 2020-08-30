/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Sampling and calculations.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/analog.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/

/* Validation Test for Current Measurement:
Using the I-SIM circuit, apply 20.0mV to the ATmega406 CCADC input.
Read back the variable LatestCCI.  Multiply this value by 53.7uV/count.
You should get approximately 20,000uV, or 20mV.  The variable LatestCCI
is intended to be used for cell impedance measurements and is not filtered
or scaled in any way, but is a copy of the raw result of the instantaneous
CC reading.

The variable CCarray contains averaged 1-second Current readings based on the
results of the Instantaneous reading of the CCADC.  These calculations
assume a 5mOhm sense resistor.  To verify these readings, pick one
sample out of the CCarray and convert to decimal.  The value should be read
directly in mA.  If using 20mV as noted above, this would correspond to
a value of 20mV/5mOhm = 4000mA, therefore all non-zero elements in the array
should read approx. 4000 (decimal).

The Accumulator should also be tested, since its reading must also be
scaled in order to be read as mA.  Assuming 20mV applied to the input,
and each bit corresponding to 1.678uV, then each 1-second reading of the
CCADC accumulated result should be approx. 20mV/1.678uV = 0x2EF8.
This number must be multiplied by 3600 to convert from seconds to hours.
Dividing this result by 10727 will yield the current in mAHr, in this
case approximately 4000mAHr.


*/





//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)
#include <inavr.h>

#include "main.h"
#include "pack.h"

#define MODULE_ANALOG
#include "analog.h"

#define MODULE_CALIBRATION
#include "calibration.h"


#include "smbus.h"
#include "timer.h"	// (for access to the Timer32KHz variable)
#include "pwrmgmt.h"
#include "ee.h"


//Local prototypes (not exposed in analog.h)
void CCarray_Init(void);




//Local variables (not exposed in analog.h)

//signed long RunningAcc = 0;		//this must be externally visible for Hibernate mode estimated power loss
signed long MaxTopAcc = 0;
signed long MaxBottomAcc = 0;

unsigned char CC_delay_acc = 0;
unsigned char CC_delay_inst = 0;

//Calculated, calibrated results
unsigned int  CellV[4] = {0};
unsigned int  OnChipTemp = 0;

//Intermediate results
unsigned int  Thermistor[4] = {0};	// this hold an Ohmic value, scaled per "FixedThermistorPullup"
unsigned int  VPA4 = 0;			// this is V(ADC4) * 5


// The following variables are used for the 1-minute (approx) average of current flow.
// We treat CCarray as a circular buffer, containing the 64 most recent samples available.
// Each sample is properly scaled (mA) but does not include temperature or offset effects.
// These functions are kept local to this file and are not made public.
signed int CCarray[64];		//positive (charging) or negative (discharging).
char CCindex = 0;
char CCvalidsamples = 0;




//Support variables
unsigned int  ADCbuffer[10];		//raw converted results
unsigned int ADCgain[4];		//gain values for Cell1-4
unsigned int  VTgain;			//gain for on-chip temperature sensor
unsigned char ADCchannel;		//visible so we can force the start channel


#define FixedThermistorPullup 1000	/* ohmic value */

/* =====================================================================================
   =====================================================================================
   ===================================================================================== */


//Public functions (exposed in analog.h)

void SetMaxTopAcc(long value)
{
  MaxTopAcc = value;
}

void FullChargeReached(void)
{
  signed long temp;

  temp = RunningAcc - MaxBottomAcc;
  RunningAcc = MaxTopAcc = temp;
  MaxBottomAcc = 0;
  SMBvariables[SMBV_BattStatus][lobyte] &= ~FULLY_DISCHARGED;
  SMBvariables[SMBV_BattStatus][lobyte] |= FULLY_CHARGED;
}



void FullDischargeReached(void)
{
  MaxTopAcc -= RunningAcc;
  RunningAcc = MaxBottomAcc = 0;
  SMBvariables[SMBV_BattStatus][lobyte] |= FULLY_DISCHARGED;
  SMBvariables[SMBV_BattStatus][lobyte] &= ~FULLY_CHARGED;
}


/* =====================================================================================
   =====================================================================================
   ===================================================================================== */


//Assorted local-support math functions

unsigned int GetVoltage(void)	//also serves as "cmd = 9"
{
  unsigned int volt;

  volt = ReadCell(1);
  volt += ReadCell(2);
  if(PACKSTACK > 2)
    volt += ReadCell(3);
  if(PACKSTACK > 3)
    volt += ReadCell(4);
  return volt;
}	


long GetCharge(void)			//in mAHrs
{
  unsigned long calc = (RunningAcc / 10727);	// 10,727 LSB/mAh, see app-note for more details
  return calc;
}


long GetCharge_mAmins(void)	   //in mAmins
{
  unsigned long calc = (RunningAcc / 179);	// 10727 / 60 ~= 179
  return calc;
}


long GetChgUntilFull_mAmins(void)	//in mAmins
{
  unsigned long calc = (MaxTopAcc - RunningAcc);
  return (calc / 179);
}


unsigned int GetMaxChg(void)	//calculate the maximum charge that the pack is presently
                                //capable of holding (not same as DESIGN capacity)
{
  unsigned long calc = MaxTopAcc / 10727;
  return (unsigned int) calc;
}



/*
unsigned int GetCapacity(void)		//designed capacity @1C, in mAHr
{
  return PACK_DESIGNCAPTYP;
}


unsigned long GetCapacity_mAmins(void)	//designed capacity @1C, in mAmins
{
  return (PACK_DESIGNCAPTYP*60);
}
*/




/* =====================================================================================
   =====================================================================================
   ===================================================================================== */

//Functions to support specific SMBus Slave READ commands

unsigned int AtRateTTF(void)	// cmd = 5
{
  signed long calc;
  unsigned int temp = (unsigned int)SMBvar_int[SMBV_AtRate];

  if((signed int) temp > 0)	//For TTF, AtRate must be POSITIVE & NON-ZERO.
  {
    if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)  //use mW
    {
      temp = 65535;			//optional mode, not implemented.
    }
    else //use mA in calculations
    {
      calc = PACK_DESIGNCAPTYP;
      calc = calc * 60;
      calc -= GetCharge_mAmins();
      temp = calc / temp;		// (mAmins / mA) = mins
    }
  }
  else	//error
    temp = 65535;

  return temp;
}


unsigned int AtRateTTE(void)	// cmd = 6
{
  signed long calc;
  unsigned int temp = (unsigned int)SMBvar_int[SMBV_AtRate];

  if((signed int) temp < 0)	//For TTE, AtRate must be NEGATIVE & NON-ZERO.
  {
    temp = -((signed int) temp);

    if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW
    {
      calc = GetCharge_mAmins() * PACK_MINV;	//this is now in uWmins
      calc = calc / 10000;			//this is now in 10mWmins
      temp = calc / temp;			// (10mWmins / 10mW) = mins
    }
    else //use mA in calculations
    {
      calc = GetCharge_mAmins();
      temp = calc / temp;			// (mAmins / mA) = mins
    }
  }
  else	//error
    temp = 65535;

  return temp;
}


unsigned int AtRateOK(void)	// cmd = 7
{
  unsigned long calc;				 //used as available capacity
  unsigned int temp = (unsigned int)SMBvar_int[SMBV_AtRate]; //used as total rate of consumption

  if((signed int) temp < 0)	//For AtRateOK, AtRate must be NEGATIVE & NON-ZERO.
  {
    temp = -((signed int) temp);

    if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
    {
      calc = Current1Sec() * GetVoltage();	//in uW (alternate: can use PACK_MINV)
      calc = calc / 10000;			//in 10mW
      temp += calc;	//combine AtRate and present load; shouldn't overflow!

      calc = GetCharge_mAmins();
      calc = calc * PACK_MINV;
      calc = calc * 6;                          //this is now in mW-10Secs
      calc = calc / 10;				//this is now in 10mW-10Secs
    }
    else //use mA in calculations
    {
      temp += Current1Sec();			//add in the present discharge rate too!
      calc = GetCharge_mAmins() * 6;		//convert to 10-second rate
    }

    if(calc > temp)
      temp = 65535;	//return TRUE, as there is enough energy.
    else
      temp = 0;		//return FALSE. We're almost dead!
  }
  else	//error
    temp = 65535;

  return temp;
}


unsigned int GetTemperature(void)	// cmd = 8
{ // Returns temperature of MEGA406, in 0.1 degrees Kelvin.
  return ReadTemperature(0);	//channel 0 is the on-chip sensor.
}



unsigned char RelativeSOC(void) 	// cmd = 13
{
  unsigned long charge = (RunningAcc * 100) / MaxTopAcc;
//  return (unsigned int) charge;
  return (unsigned char) charge;
}


unsigned int AbsoluteSOC(void)		// cmd = 14
{
  unsigned long charge = (GetCharge() * 100);
  charge = charge / PACK_DESIGNCAPTYP;
  return (unsigned int) charge;
}


unsigned int RemainingCap(void)		// cmd = 15
{ //! \todo  This implementation does not take into account C/5 vs 1C capacity delta.
  unsigned long calc;
  unsigned int charge = GetCharge();	//in mAH

  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
  {
    calc = charge * GetVoltage();	//in uWH
    charge = calc  / 10000;		//in 10mWH
  }

  return charge;
}



unsigned int FullChgCap(void)		//cmd = 16
{ //! \todo  This implementation does not take into account C/5 vs 1C capacity delta.
  unsigned long calc = GetMaxChg();

  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
  {
    calc = calc * PACK_NOMINALV;
    calc = calc / 10000;
  }

  return (unsigned int) calc;
}


// Pass in 0 for 1-sec basis, 1 for 1-minute avg'd basis.
unsigned int TimeToEmpty(unsigned char avgd)	//cmd = 17,18; how many mins until battery is discharged at present rate
{
  signed int rate;
  unsigned long presentrate;
  unsigned long cap;

  if(0 == avgd)
    rate = Current1Sec();
  else
    rate = CCarray_Average();

  if(rate >= 0)
    return 65535;

  rate = -rate;

  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
  {
    //First, determine actual rate of Wattage being used.
    presentrate = rate * GetVoltage();		//this is in uW scale

    //Next, determine capacity at pack's minimum V (full discharge)
    cap = GetCharge_mAmins() * PACK_MINV;	//this is in uWmins scale

    //Divide uWmins by uW and you get minutes.
    cap = cap / presentrate;
  }
  else
  {
    cap = GetCharge_mAmins() / rate;	// (mAmins / mA) = minutes
  }

  return (unsigned int) cap;
}




unsigned int AvgTimeToFull(void)	//cmd = 19
{
  signed int rate;
  unsigned long presentrate;
  unsigned long cap;

  rate = Current1Sec();
  if(rate <= 0)
    return 65535;


  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
  {
    //First, determine actual rate of Wattage being used.
    presentrate = rate * GetVoltage();		//this is in uW scale

    //Next, determine capacity at pack's minimum V (full discharge)
    cap = GetChgUntilFull_mAmins() * PACK_MINV;	//this is in uWmins scale

    //Divide uWmins by uW and you get minutes.
    cap = cap / presentrate;
  }
  else
  {
    cap = GetChgUntilFull_mAmins() / rate;	// (mAmins / mA) = minutes
  }

  return (unsigned int) cap;
}







/* =====================================================================================
   =====================================================================================
   ===================================================================================== */


//Local 'helper' functions (not exposed in analog.h)


//! \todo  NOTE: the signature/fab.calibration bytes definitions assume Rev *E* silicon ONLY.
//! Earlier revisions will require changes.
#define FAST_RC_CAL    (sig_array[0x01])
#define SLOW_RC_LO_CAL (sig_array[0x06])
#define SLOW_RC_HI_CAL (sig_array[0x07])
#define BG_C_CAL       (sig_array[0x09])
#define CELL1_LO_CAL   (sig_array[0x10])
#define CELL1_HI_CAL   (sig_array[0x11])
#define CELL2_LO_CAL   (sig_array[0x12])
#define CELL2_HI_CAL   (sig_array[0x13])
#define CELL3_LO_CAL   (sig_array[0x14])
#define CELL3_HI_CAL   (sig_array[0x15])
#define CELL4_LO_CAL   (sig_array[0x16])
#define CELL4_HI_CAL   (sig_array[0x17])
#define ADC0_LO_CAL    (sig_array[0x18]) // currently not used
#define ADC0_HI_CAL    (sig_array[0x19]) // currently not used
#define VPTAT_LO_CAL   (sig_array[0x1A])
#define VPTAT_HI_CAL   (sig_array[0x1B])

// Old defines (for rev.D), not fully compatible with new routines
/*
#define FAST_RC_CAL  (sig_array[0x01])
#define SLOW_RC_CAL  (sig_array[0x03])
#define BG_C_CAL     (sig_array[0x09])
#define BG_R_CAL     (sig_array[0x0B])
#define VPTAT_HI_CAL (sig_array[0x0D])
#define VPTAT_LO_CAL (sig_array[0x0F])
#define CELL1_CAL    (sig_array[0x11])
#define CELL2_CAL    (sig_array[0x13])
#define CELL3_CAL    (sig_array[0x15])
#define CELL4_CAL    (sig_array[0x17])
*/


void ReadFactoryCalibration(void)
{
  unsigned char __flash * ptr = 0;
  char i;
  char flags = SREG;
  char temp;
  unsigned char sig_array[0x1C];   //Array with signature/fab.calibration bytes


  __disable_interrupt();
  for(i=0; i< 0x1C; i++)
  {
    SPMCSR = (1<<SIGRD) | (1<<SPMEN);
    temp = *ptr++;
    sig_array[i] = temp;
  }

  if(flags & 0x80)
    __enable_interrupt();

  //Now apply the Cal values we just read out.

//  SlowRCCal = SLOW_RC_CAL;
  SlowRCCal = (SLOW_RC_HI_CAL << 8)|(SLOW_RC_LO_CAL);
  FastRCCal = FAST_RC_CAL;

  //Calibrate bandgap (fab. or battery factory)
  BGCCRCal = BG_C_CAL| 0x80;
  if (ReadVrefCalibration()) {
	calibration_state &= ~CAL_VREF_MASK;
	calibration_state |=  CAL_VREF_OK;
  } else {
    calibration_state &= ~CAL_VREF_MASK;
  }

  //Save the Cell1-Cell4 gain values
  ADCgain[0] = (CELL1_HI_CAL << 8)|(CELL1_LO_CAL);
  ADCgain[1] = (CELL2_HI_CAL << 8)|(CELL2_LO_CAL);
  ADCgain[2] = (CELL3_HI_CAL << 8)|(CELL3_LO_CAL);
  ADCgain[3] = (CELL4_HI_CAL << 8)|(CELL4_LO_CAL);

  //On-chip temperature sensor calibration value
  VTgain = (VPTAT_HI_CAL<<8) | VPTAT_LO_CAL;	//on-chip temp sensor

  // Calibration values for CCoffset and CCIoffset?
  if (ReadCCOffsetCalibration()) {
	calibration_state &= ~CAL_CC_MASK;
	calibration_state |=  CAL_CC_OK;
  } else {
    calibration_state &= ~CAL_CC_MASK;
  }
}



unsigned char ReadVrefCalibration(void)
{
  unsigned char temp;

  while(EECR & (1<<EEPE));
  EEAR = EESTORAGE_BGCCR;
  EECR = (1<<EERE);	//read
  temp = EEDR;
  if (temp != 0xFF) {
    BGCCR = temp;
    while(EECR & (1<<EEPE));
    EEAR = EESTORAGE_BGCRR;
    EECR = (1<<EERE);	//read
    BGCRR = EEDR;
    return(1);
  } else {
    BGCRR = 0x0F;
    BGCCR = BGCCRCal;		// signature value (with Band gap enabled)
    return(0);
  }
}


unsigned char ReadCCOffsetCalibration(void)
{
  signed char temp;

  while(EECR & (1<<EEPE));
  EEAR = EESTORAGE_CC_valid;
  EECR = (1<<EERE);	//read
  temp = EEDR;
  if (temp != -1) {
    do {} while(EECR & (1<<EEPE));
    EEAR = EESTORAGE_CCoffset;
    EECR = (1<<EERE);	//read
    temp = EEDR;
    CCoffset = (signed int)temp;
    do {} while(EECR & (1<<EEPE));
    EEAR = EESTORAGE_CCIoffset;
    EECR = (1<<EERE);	//read
    temp=EEDR;
    CCIoffset = (signed long)temp;
    return(1);
  } else {
    CCIoffset = 0;
    CCoffset  = 0;
    return(0);
  }
}

/* =====================================================================================
   =====================================================================================
   ===================================================================================== */


/* Calibration routines for VREF and CC/CCIoffset
*/



unsigned char CalibrateVREF(void)         // calibration of VREF
{
  long int V_error;                       // Variable to keep error after conversion
  long int old_V_error;                   // Value used to keep old error value during linear scan
  unsigned char ratio_counter;            // Value used to count up the BGCCR value
  unsigned char loopcount;                // Value used during temperature calibration
  volatile unsigned int vui_temp;         // Dummy variable used to ensure correct reading of VADC

  BGCRR = 0x0f;							  // Load Factory calibration
  BGCCR = BGCCRCal;  			          // Load Factory calibration (with Band gap enabled)
  VADMUX = CAL_CHANNEL;                   // Select terminal input defined in calibration.h
  VADCSR = (1 << VADEN);                  // Enable the VADC
  vui_temp = VADC;                        // Dummy read(s) to ensure proper operation
  loopcount = CAL_WAIT;				      // Wait for error from CellBalancing to cancel
  while (loopcount--) {
    VADCSR |= (1 << VADSC) | (1 << VADCCIF);// Start a VADC conversion and clear any pending interrupts
    do {} while(!(VADCSR & (1 << VADCCIF)));// Wait while conversion in progress
    vui_temp = VADC;                      // Dummy read(s) to ensure proper operation
  }
  VADCSR |= (1 << VADSC) | (1 << VADCCIF);// Start a VADC conversion and clear any pending interrupts
  do {} while(!(VADCSR & (1 << VADCCIF)));// Wait while conversion in progress
  V_error = VADC;                   	  // Calculate the error of the measured value
  V_error = V_error * CAL_GAIN;
  V_error = V_error - Vcalibration_value;
  loopcount = 8;						  // Check for all different test limits
  while (V_error < vcalibration_level[loopcount]) {
    loopcount--;
    if (!loopcount) {
      break;
    }
  }
  BGCRR = tempcal[loopcount];             // Set BGCRR to correct setting

  V_error = vcalibration_level[7];        // Init old_V_errror
  ratio_counter = (1 << BGEN) - 1;        // Reset ratio counter (keep Band gap enabled), -1 because of increment
  do                                      // Repeat until VADC value becomes larger than reference
  {
    ratio_counter++;                      // Select next BGCCR value;
    BGCCR = ratio_counter;                // Set new BGCCR value
    old_V_error = V_error;                // Calulate the error of the measured value
    if(ratio_counter & 0x40)              // Ratio_counter bit 6 set means calibration failed
    {
      BGCRR = 0x0f;						  // Load Factory calibration
      BGCCR = BGCCRCal; 			      // Load Factory calibration (with Band gap enabled)
      return 0;                           // Return with fail flag
    }
    vui_temp = VADC;                      // Dummy read to ensure proper operation
    VADCSR |= (1 << VADSC) | (1 << VADCCIF);// Start a VADC conversion and clear any pending interrupts
    do {} while(!(VADCSR & (1 << VADCCIF)));// Wait while conversion in progress
    V_error = VADC;                   	  // Calulate the error of the measured value
    V_error = V_error * CAL_GAIN;
    V_error = V_error - Vcalibration_value;
  } while(V_error > 0);                   // Loop until VADC output is larger than the reference setting
  if(old_V_error < (-V_error)) {          // Select the best value out of the last two BGCCR values
    ratio_counter--;                      // Value below the reference setting is closest to the reference
  }
  BGCCR = ratio_counter;                  // Set the correct BGCCR value

  do {} while(EECR & (1<<EEPE));
  do {} while(SPMCSR & (1<<SPMEN));
  EEAR = EESTORAGE_BGCRR;
  EEDR = tempcal[loopcount];
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go

  do {} while(EECR & (1<<EEPE));
  EEAR = EESTORAGE_BGCCR;
  EEDR = ratio_counter;
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
  EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go

  return 1;
}



unsigned char CalibrateCCoffset(void)         // calibration of CC offset
{                                       // assumes 0 current through shunt for minimum 1 second before calibration
  unsigned char index = CCindex;
  volatile signed long cc_temp;         // for dummy reads

  index--;
  index &= 63;
  CCIoffset = CCarray[index];

  ChangePowerMode(POWERMODE_ACTIVE,0);
  while (CC_delay_acc--) {  // check if we need to discard samples
    do {} while (!(CADCSRB & (1<<CADACIF)));
    cc_temp = CADAC;
    CADCSRB = (1<<CADACIE) | (1<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
  }
  CCoffset = CADAC;

  if ((CCoffset > CCoffset_limit) || (CCIoffset > CCIoffset_limit) ||
      (CCoffset < -CCoffset_limit) || (CCIoffset < -CCIoffset_limit)) {  // Max offset limits to avoid wrong calibration
    CCoffset = 0;
    CCIoffset = 0;
    return(0);

  } else {
    do {} while(EECR & (1<<EEPE));
    do {} while(SPMCSR & (1<<SPMEN));
    EEAR = EESTORAGE_CC_valid;
    EEDR = 0x00;
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go

    do {} while(EECR & (1<<EEPE));
    EEAR = EESTORAGE_CCIoffset;
    EEDR = (unsigned char)CCIoffset;
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go

    do {} while(EECR & (1<<EEPE));
    EEAR = EESTORAGE_CCoffset;
    EEDR = (unsigned char)CCoffset;
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
    EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go
    return(1);
  }
}



/* =====================================================================================
   =====================================================================================
   ===================================================================================== */



/* General description of Coulomb Counter usage:
 *
 * The Coulomb Counter must be running whenever the chip is not
 * in deep sleep mode.  The interrupt is used to perform software
 * accumulation & must also handle analog offset voltage correction
 * for the input stage.
 *
*/

void CCmode(unsigned char mode)
{
  switch (mode)
  {
    default:
    case CC_DISABLED:
      CCSR = 0;					//switch to the Slow RC Oscillator to save power
      CCarray_Init();
      CADCSRA = 0;
      break;

    case CC_ACCUMULATE:				//enable both Accumulate and Instantaneous
      if(CCSR != ((1<<XOE)|(1<<ACS)))		//if the 32KHz xtal isn't already in use, start it up.
      {
        Timer32KHz = 9;
        CCSR = (1<<XOE);
      }
      BGCCR |= (1<<BGEN);			//ensure that the bandgap is enabled
      CADCSRB = (1<<CADACIE) | (1<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
      while ( CADCSRA & (1<<CADUB) );		//wait for clock domain sync
      CADCSRA = (1<<CADEN) | ACCUM_CONV_TIME;	//! \todo  See ANALOG.H to choose your interval for CC_ACCUMULATE
      CC_delay_acc  = 4;			//must ignore the first 4 readings!
      CC_delay_inst = 4;			//must ignore the first 4 readings!
      CCarray_Init();
      break;

    case CC_REGULAR:				//enable ONLY Regular mode is enabled
      if(CCSR != ((1<<XOE)|(1<<ACS)))		//if the 32KHz xtal isn't already in use, start it up.
      {
        Timer32KHz = 9;
        CCSR = (1<<XOE);
      }
      CCarray_Init();
      BGCCR |= (1<<BGEN);			//ensure that the bandgap is enabled
      CADRDC = -(ACTIVE_CURRENT_THRESHOLD + (CCIoffset>>4));
      CADCSRB = (1<<CADRCIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
      while ( CADCSRA & (1<<CADUB) );		//wait for clock domain sync
      CADCSRA = (1<<CADSE) | REG_CONV_TIME;	//! \todo  See ANALOG.H to choose your interval for CC_REGULAR
      break;
  }
}


void CCinit(void)
{
  CCmode(CC_DISABLED);
}



/* =====================================================================================
   =====================================================================================
   ===================================================================================== */





//Call this at startup to ensure the buffer is cleared.
void CCarray_Init(void)
{
  CCvalidsamples = 0;			//reset the # of valid samples
  CCindex = 0;				//reset the insertion index
}


//This inserts a new sample into the circular buffer.
void CCarray_AddSample(signed int newsample)
{
  if(CC_delay_inst)
  {
    CC_delay_inst--;
    return;
  }

  CCarray[CCindex++] = newsample;
  CCindex &= 63;

  if(CCvalidsamples < 64)
    CCvalidsamples++;
}


//This retrieves the most recent sample out of the circular buffer.
signed int Current1Sec(void)	//! \todo  NOTE: This could produce a bad result within the first 1Sec of operation.
{
  unsigned char temp = CCindex;

  temp--;
  temp &= 63;
  return CCarray[temp];
}





//Note: the data source here is INSTANTANEOUS CURRENT readings, not the Accumulator.
//  Although it is not as precise as the Accum, it is accurate within 0.5% of full-scale.
signed int CCarray_Average(void)	//this will accurately reflect up to 64 seconds of data.
{					
  signed long avg = 0;
  unsigned char ctr = CCvalidsamples;	//grab a local copy since CCvalidsamples is modified by an INTERRUPT
  unsigned char temp;
  signed int * ptr = CCarray;		//the same as saying  &CCarray[0]

  if(0 == ctr)
    return 0;

  temp = ctr;
  while(temp--)				//add up ONLY the valid samples!
    avg += *ptr++;

  if(64 == ctr)
    return (signed int) (avg >> 6);	//use this optimized version after the first 64 seconds
  else
  if(1 == ctr)
    return (signed int) (avg);
  else
    return (signed int) (avg / ctr );	//this takes longer due to division.
}



/* ******************************************************************** *
 * Coulomb Counter, Instantaneous conversion complete ISR
 *
 * This interrupt fires after every conversion, at 3.9ms intervals.
 * Every 256 interrupts is therefore 1 second.
 *
 * Each count of the result is 53.7uV, which, into a 5mOhm sense R,
 * indicates 10.74mA flowing.  This is quick-responding and can therefore
 * be used in conjunction with cell voltage measurements to calculate
 * cell impedance by watching the delta-V on the cells when compared to
 * any available delta-I.  There obviously must be sufficient delta-I
 * to permit a reasonably accurate division result, otherwise the
 * calculation could be way off.  Note that the CC ADC is separate
 * hardware from the 12-bit general-purpose ADC and can therefore do a
 * current measurement in parallel with one cell voltage measurement.
 * There is a global variable, LatestCCI, that always contains the most
 * recent value.  The intent is that this value would be captured along
 * with the individual cell voltage at a given instant.
 *
 * ( NOTE: if you use LatestCCI, remember that it is NOT SCALED TO mA! )
 *
 * We also use this as the basis of the required 1-minute averaged current.
 * We play some tricks to get this value without doing ugly math.
 * Since the values need to be available in milliamps, we must accumulate
 * lots of samples to make up for the fact there's over a 10X scale delta.
 * Specifically, we get 1 count for every 10.74mA.  If we accumulate
 * 10.74 x 16 = 172 samples, then we can simply divide by 16 to get a
 * 1mA/step average result. We will thus take 172 samples per second.
 * (Note: this value could be modified SLIGHTLY to effect a 'calibration'
 *  without adding additional math processing burden.)
 *
 * The next problem is to spread these samples out evenly over the sampling
 * period (1 second).  To do this, we use a simple first-order sigma-delta
 * software modulator; if its 1-bit output is a '1', then we accumulate the
 * sample that's available at this moment.
 *
 * ******************************************************************** */



#define CCI_CAL 172	/* # of samples we need to accumulate in a second */

#pragma vector = CCADC_vect
__interrupt void CC_Instantaneous_ISR(void)
{
  static unsigned char timer = 0;	//256 interrupts = 1 second
  static signed long sl = 0;		//averaging accumulator
  static unsigned char mod_remainder;	//used by sigma-delta modulator
  signed int temp,temp2;



  if(PowerMode == POWERMODE_IDLE)	//do Regular Current mode instead.
  {
    temp = CADIC;
    temp <<= 3;				//note: CADIC is actually 13 bits incl. sign
    RunningAcc += (signed long) temp;	//update the charge state
    RunningAcc += (signed long) temp;	//update the charge state
    RunningAcc += (signed long) temp;	//update the charge state
    RunningAcc += (signed long) temp;	//must do it 4 times (Instantaneous * 32 = Accumulate)

    //Note: it is POSSIBLE to overflow here, but not likely unless pushing lots of current.
    temp2 = CADIC;
    temp = temp2;
    temp += temp2;
    temp += temp2/2;			//10.5x, a good approximation of 10.7x
    temp -= CCIoffset;			//subtract the offset calibration value
    CCarray_AddSample(temp);		//update the 64-second average
    return;
  }

  //If get here, we're running in Active mode, so we use this only for 1-second avg current reading.
  if(0 == ++timer)			//one per second, pull out the accumulated
  {					// result and update the 64-second list.
    sl /= 16;				//divide result by 16, per our math.
    sl -= CCIoffset;        //subtract the offset calibration value
    CCarray_AddSample((signed int) sl);	//extract the properly scaled current value.
    sl = 0;				//clear the accumulator
  }

  // Now, accumulate SOME of the available samples, specifically, (10.74 x 16) of them,
  //  to ultimately yield a 1mA-per-bit current measurement in the accumulator variable "sl".

  temp = mod_remainder;			//(this auto-clears the upper byte, by promotion.)
  temp += CCI_CAL;			//if this addition causes a CARRY, then USE this sample.
  mod_remainder = (unsigned char) temp;	//Re-save.

  if(temp>>8)	//was a CARRY generated?
  {
    temp = CADIC;			//grab the result
//    temp -= CCIoffset;     // remove offset from each sample, different value than the one currently used/calibrated
    LatestCCI = temp;			//update the globally-available value
    sl += temp;
  }
  else
    LatestCCI = CADIC;			//update the globally-available value
}




/* =====================================================================================
   =====================================================================================
   ===================================================================================== */






/* ******************************************************************** *
 * Coulomb Counter, Regular Current ISR
 *
 * This interrupt is tripped when the discharge current exceeds the amount
 * specified in the CADRDC register (set up by the CCmode() function).
 * When this fires, it indicates that the system needs to switch back to
 * Active Mode.
 *
 * ******************************************************************** */

#pragma vector = CCADC_REG_CUR_vect
__interrupt void CC_RegularCurrent_ISR(void)
{
  ChangePowerMode(POWERMODE_ACTIVE,0);
}








/* =====================================================================================
   =====================================================================================
   ===================================================================================== */


/* ********************************************************************
 * Coulomb Counter, Accumulator ISR
 *
 *! \todo
 * This interrupt is the main mechanism for collecting charge estimates
 * for the 'fuel gauge' function.  Note that CCADC offset error must also
 * be corrected here, and temperature should be taken into account in
 * subsequent calculations as well.
 *
 * ******************************************************************** */

//unsigned char ccindex = 0;
//long ccarray[256];

#pragma vector = CCADC_ACC_vect
__interrupt void CC_Accumulator_ISR(void)
{
  union {signed long acc; unsigned char byte[4];} lastCCreading;

//! \todo If using IAR3.20, the following lines need to be enabled and the one below disabled.
/*
  lastCCreading.byte[0] = CADAC0;	//grab the ACC value
  lastCCreading.byte[1] = CADAC1;	//grab the ACC value
  lastCCreading.byte[2] = CADAC2;	//grab the ACC value
  lastCCreading.byte[3] = CADAC3;	//grab the ACC value
*/
  lastCCreading.acc = CADAC;  // Only works with IAR 4.10 and later

  lastCCreading.acc -= (signed long) CCoffset;	//although the CC's offset is temperature-dependent
  				// we have NOT implemented thermal compensation.

  if(CC_delay_acc)
  {
    CC_delay_acc--;
    return;
  }

  RunningAcc += lastCCreading.acc;	//merge this sample with the main accumulator.

}





/* =====================================================================================
   =====================================================================================
   ===================================================================================== */






/* General description of ADC usage:
 *
 * The ADC handles 10 different inputs.  We trigger a periodic
 * scan through all the channels back-to-back by a timer.
 * Then, when desired, the user can request a particular measurement,
 * according to its function group, and will get back a properly
 * scaled and calibrated value.  For instance, reading a temperature
 * will scale the ADC result including gain and offset; when reading
 * a particular cell's voltage, it will be corrected according to the
 * offset and gain for that particular channel.
 *
 * When reading cell voltages, we must ensure that cell balancing FETs
 * are disabled.
 *
 * The time required for fully charging a cell's input bypass cap to
 * within 1/4000th of full-scale (about 1mV error, max.) is approx.
 * 8 time-constants (not 9, since the cap starts at cellV/2 rather
 * than at 0.00V).  In the configuration we use, we have a 0.1uF with
 * a 500ohm resistor, giving a TC of 50uS.  400uS is therefore required
 * before commencing a reading of any cell's ADC channel.  We accomplish
 * this delay without cost by reading the other channels first, THEN
 * reading the cell inputs.
 *
*/






void ADCinit(void)
{
//  char i;

  ReadFactoryCalibration();

  // Note: SLOW_RC_CAL is used in calculations on the Wakeup timer to
  // calculate the elapsed time, rather than to adjust its oscillator.
}




//This routine starts a scan of all 10 ADC channels back-to-back.
// The parameter specifies which Thermistor input to read (0-3).
void StartAdc(unsigned char select)
{
  unsigned char temp = (1<<select);


  //! \todo  NOTE: If not using ADC0-3 exclusively as analog inputs, you MUST modify the following code.
  DIDR0 = 0x0F;						//disable ADC0-3 digital input buffer

  PORTA &= 0xE0;
  PORTA |= (1<<4);		//drive PA4 high!
  DDRA  &= 0xE0;
  DDRA  |= (1<<4) | temp;	//make PA4 and the selected signal be Outputs

  VADMUX = 5;						//set up for channel 5 first.
  VADCSR = (1<<VADEN) | (1<<VADCCIF);			//clear any pending int
  VADCSR = (1<<VADEN) | (1<<VADSC) | (1<<VADCCIE);	//Start the first conversion & ena int's
}



// Automatically scan through all ADC channels.
// When all 10 are done, this interrupt disables itself.
#pragma vector = VADC_vect
__interrupt void ADC_INT(void)
{
  unsigned char temp;
  static char counter;	//added to handle Rev E silicon errata.

  temp = VADMUX;

  ADCbuffer[temp-1] = VADC;	//save the result

  if((temp <= 4) && (temp >= 1))	//just read a Cell?
    cell_current[temp-1] = LatestCCI;	//save its associated CCI reading for impedance.

  if(temp == 5) {			// init counters before getting there during scan.
    counter = VPTAT_READINGS; // per errata for Rev E silicon
    temp++;
  }

  if((temp == 6)||(temp == 7)) {  //
    if(--counter == 0) {		  // wait for VPTAT to stabilize, per errata for rev.E
      temp++;
      counter = ADC0_READINGS;    // wait for VREF to stabilize after VPTAT readings
    }
  } else {
    temp++;
    if(10 == temp) {         // earliest possible 8, latest 10, if earlier use 6/7 ifs above
      DisableCellBalancing();	//if more than 519uS is needed, we can do this earlier than 10!
    } else {
      if(10 < temp) {			// continue with cell1-4?
        temp = 1;
      } else {
        if(5 == temp) {	  //have we scanned ALL channels now?
          SetADCScanDone;		//flag Mainline that we have new samples!
          EnableCellBalancing();
          VADCSR = 0;		//disable this ADC and its Interrupt.
          return;
        }
      }
    }
  }

  VADMUX = temp;
  VADCSR |= (1<<VADSC);		//start next conversion now.
}




//This should be called from main().
//After an ADC scan, this will calculate new results.
//This needs to know the state from the thermistor state machine.
void CalculateADCresults(void)
{
  unsigned long calc;
  unsigned char index;
//  unsigned int fixedV;
  unsigned int thermV;

  //Calculate new cell voltages
  for(index = 0; index < PACKSTACK; index++)
  {
    calc = ADCbuffer[index]<<2;
    calc = calc * ADCgain[index];
    calc >>= 16;
    CellV[index] = (unsigned int)(calc);
    if((unsigned int)calc < CELL_TOOLITTLEV)
      DoShutdown(SHUTDOWN_REASON_UNDERVOLTAGE);		//go to Shutdown Mode if cell is drained!
    if((unsigned int)calc > CELL_TOOMUCHV)
      DoShutdown(SHUTDOWN_REASON_OVERVOLTAGE);		//go to Shutdown Mode if cell is too full!
  }

  //! \todo  Code could be added HERE to do something with the cell_current[] array,
  //!        for the purpose of determining cell impedances.

  //Calculate new ADC4 (PA4) reading due to its scale factor of ~0.2X
  calc = ADCbuffer[4];
  calc = calc * 344;
  VPA4 = (unsigned int) (calc >> 8);


  //Calculate on-chip temperature. There may be ways to optimize this,
  // such as pre-scaling VTgain up by 2.5X, then scaling ADCbuffer[6] by 4X,
  // so that when multiplied you get 10X. Can then shift the result UP by 2,
  // then grab the upper two bytes as the result. Not sure if VTgain can be
  // scaled by 2.5X and still fit in an INT.  Doing this optimization would save a long div.
  calc = ADCbuffer[5];
  calc = calc * VTgain;			//this gives 'K * 2^14
  OnChipTemp = (unsigned int) (calc / 1638);	//divide by (2^14 / 10) to get 0.1'K


  //Update the result for the thermistor that was just checked.
  index = ThermistorSelect;	//ThermistorSelect says which output was low.
  index++;			//Go to next channel & use that reading as midpoint.
  index &= 0x03;
  thermV = ADCbuffer[index+7];
  Thermistor[ThermistorSelect] = thermV;
}




// Returns on-chip temperature in 0.1 degrees Kelvin.
// Returns Thermistor resistance reading directly in Ohms (change this as needed).
// channel: 0=on-chip; 1-4 = PA0-3 thermistors
unsigned int ReadTemperature(unsigned char channel)
{

  if(0 == channel)
    return OnChipTemp;

//  temperature = ADCbuffer[6] * VTgain;
//  temperature = temperature / 1638;		// =16384/10
//    temperature -= 2731;			//convert from 'K to 'C (0'C = 273.15K)
//    return (unsigned int) temperature;


  if(channel > 5)
    return 0;				//choose what error value you want.

  //! \todo   Insert calculations here to convert voltage reading(s) to temperature(s).
  return Thermistor[channel-1];		//for now, just return the VADC result

}



//Read the cell voltage in millivolts (always positive). (cell = 1-4)
unsigned int ReadCell(char cell)
{
  if((cell == 0) || (cell > 4))
    return 0;

  return CellV[cell-1];
}


void DisableCellBalancing(void)
{
  CBCR = 0;
}


void EnableCellBalancing(void)	//if CellToBalance = 0, no balancing req'd.
{
  if(CellToBalance)
  {
    CBCR = (1<<(CellToBalance-1));
  }
  else
    CBCR = 0;
}




