/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Main
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/main.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)
#include <inavr.h>

#define MODULE_MAIN
#include "main.h"
#include "analog.h"
#include "calibration.h"
#include "timer.h"
#include "pack.h"
#include "smbus.h"
#include "gpio.h"
#include "safety.h"
#include "pwrmgmt.h"
#include "ee.h"



//Local Prototypes
void BalanceCheck(void);
void ChargeCheck(void);
void ThermalCheck(void);
void AlarmModeDisabledCheck(void);
void AlarmConditionsCheck(void);
void MasterSMBusCheck(unsigned char ResetTimer);
void SleepMgr(void);
void WdogMgr(void);
void DoCalibrate(void);



void InitAll(void)
{
  InitSMBus();		//set this up first, as spec req's 1mS wakeup!

//! \todo  Re-enable the Wdog for your final code.
//! It is disabled here to allow debugging to take place without interference.
//
//  Wdog_init(WDOG_MODE_RSTINT, WD1sec);		//set rate to slow


  PinInit();		//set up I/O
  T0init();
  T1init();
  PBinit();		//activate Pushbutton Interrupt
  CCinit();		
  ADCinit();		
  HWProtectinit();
  InitSMBvariables();
}



unsigned char qtrsectick = 3;
unsigned char AlarmModeDisabledTimer = 0;


//#pragma object_attribute=__c_task
void main(void)
{
  InitAll();
  ChangePowerMode(POWERMODE_POWERSAVE,0);
  __enable_interrupt();

  LEDs = 0x1f;

  for(;;)
  {
    if(action_flags)	//see what needs to be handled right now
    {
      if(CheckCalibRequest)
      {
        ClrCalibRequest;
        DoCalibrate();        // Perform requested calibration.
      }

      if(CheckADCScanDone)
      {
        ClrADCScanDone;

        //Runs approx. once per second, AFTER new VADC readings are available.
        CalculateADCresults();		// produce scaled results from raw ADC samples
        BalanceCheck();			// see if we need to do any cell balancing
        ChargeCheck();			// monitor charging/discharging
        ThermalCheck();			// watch for over-temperature conditions
        AlarmModeDisabledCheck();	// start timer for re-enabling Alarm Mode
        AlarmConditionsCheck();		// see if any capacity-related alarm conditions exist
        MasterSMBusCheck(0);		// see if anything needs to be sent out
      }

      if(CheckQtrSec)
      {
      	ClrQtrSec;
      	SetGenericTimer(OneQtrSecond, 244);

        WdogMgr();

        //Check if we're timing the startup of the 32KHz oscillator.
        if(Timer32KHz)
        {
          if(--Timer32KHz == 0)
            CCSR = ((1<<XOE)|(1<<ACS));
        }
      	
      	if(++qtrsectick >= 4)
      	{ //execute the following ONCE PER SECOND
      	  qtrsectick = 0;
      	  ThermistorSelect++;
      	  ThermistorSelect &= 0x03;
      	  StartAdc(ThermistorSelect);
      	}
      }

      if(CheckAlarmMode)	//if bit is turned on, start 60-sec timer to re-enable it
      {
        ClrAlarmMode;			//clear the ACTION flag, not the bit in BATTERY_MODE.
        AlarmModeDisabledTimer = 0;	//clear 60 sec timer
      }

      if(CheckMasterSMBdone)
      {
      	ClrMasterSMBdone;
        SMBvariables[SMBV_BattStatus][lobyte] &= ~(0x0F);  //clear Error Code after TX
        MasterSMBusCheck(1);				   //reset the inter-msg timer
      }
    }
    else //if no ActionFlag bits are set
    {
      if(!GetGenericTimer(OneQtrSecond))	//this should be running always!
        SetGenericTimer(OneQtrSecond, 244);	//restart 250mS timer if it got shut off!
    }

    SMB_CmdInterpreter();    			//See if there were any received commands.
    Check50uS();				//this checks whether we are trying to grab the bus.
    SetLEDs(LEDs);				//change the parameter to be whatever you want.

//    SleepMgr();	//! \todo  This is disabled ONLY for debugging purposes.

  }
}



//Check the individual cell readings and see if they are diverging.
void BalanceCheck(void)
{
  unsigned int Vmax = ReadCell(1);
  unsigned int temp;
  unsigned char maxcell = 1;
  unsigned char delta = 0;

  //First determine which cell is the highest, and by how much.
  temp = ReadCell(2);
  if(temp > Vmax)
  {
    delta += (temp - Vmax);
    Vmax += delta;
    maxcell = 2;
  }
  else
  if(temp < (Vmax - delta))
  {
    delta = (Vmax - temp);
  }

  if(PACKSTACK > 2)		//this is in  pack_cfg.h
  {
    temp = ReadCell(3);
    if(temp > Vmax)
    {
      delta += (temp - Vmax);
      Vmax += delta;
      maxcell = 3;
    }
    else
    if(temp < (Vmax - delta))
    {
      delta = (Vmax - temp);
    }
  }

  if(PACKSTACK > 3)
  {
    temp = ReadCell(4);
    if(temp > Vmax)
    {
      delta += (temp - Vmax);
      Vmax += delta;
      maxcell = 4;
    }
    else
    if(temp < (Vmax - delta))
    {
      delta = (Vmax - temp);
    }
  }

  //We now know which cell is the highest, and what the delta is.

  //If the imbalance is high enough, start Balancing.
  if(delta > MAX_IMBALANCE)
  {
    CellToBalance = maxcell;	//save which cell is the highest
    EnableCellBalancing();
  }
  else	//ensure that Balancing is OFF.
  {
    CellToBalance = 0;		//You MUST do this; the ISR enables Balancing, so if
    DisableCellBalancing();     //you leave this non-zero, it WILL re-assert balancing.
  }
}







void ChargeCheck(void)	//monitor charging and discharging
{
  signed int cell = (signed int) ReadCell(1);	//doesn't really matter which cell...
  unsigned char charge_percent;

  //First, the voltage-based assessment.
  if(cell >= CELL_NOMINALV)
  {
    if(SMBvariables[SMBV_BattStatus][lobyte] & FULLY_CHARGED)	//already had reached this point?
    {
      if(Current1Sec() < 0)	//started discharging yet?
      {
        SMBvariables[SMBV_BattStatus][lobyte] |= DISCHARGING;
        FCSR = (1<<DFE) | (1<<CFE);	//enable both FETs
      }
      else //not discharging yet
      {
        if((cell >= CELL_TOOMUCHV) && (Current1Sec() > 0))	//still CHARGING??
        {
          FCSR = (1<<DFE) | (1<<PFD);	//stop charging (CAN STILL DISCHG THRU DIODE!
        }
      }
    }
    else		//initial detection of FULL condition
    {
      FullChargeReached();		//update Capacity figures.
      SMBvariables[SMBV_BattStatus][lobyte] |= FULLY_CHARGED;

      if(Current1Sec() > 0)		//still charging...
      {
        SMBvariables[SMBV_BattStatus][hibyte] |= TERMINATE_CHARGE_ALARM;
        SMBvariables[SMBV_BattStatus][hibyte] |= OVER_CHARGED_ALARM;
      }
    }
    return;		//don't bother running any further tests!
  }
  else
  if(cell <= CELL_MINV)
  {
    if(SMBvariables[SMBV_BattStatus][lobyte] & FULLY_DISCHARGED)  //already had reached this point?
    {
      if(Current1Sec() > 0)					//started CHARGING yet?
      {
        SMBvariables[SMBV_BattStatus][lobyte] &= ~DISCHARGING;
        FCSR = (1<<DFE) | (1<<CFE);				//enable both FETs
      }
      else //not charging yet
      {
        if((cell <= CELL_TOOLITTLEV) && (Current1Sec() < 0))	//still DISCHARGING??
        {
          FCSR = 0;	//turn off Chg & Dischg FETS, only Precharge FET enabled.
          SMBvariables[SMBV_BattStatus][hibyte] |= TERMINATE_DISCHARGE_ALARM;
          SMBvariables[SMBV_BattStatus][lobyte] |= DISCHARGING;
        }
      }
    }
    else		//initial detection of EMPTY condition
    {
      FullDischargeReached();		//update Capacity figures.
      SMBvariables[SMBV_BattStatus][lobyte] |= FULLY_DISCHARGED;
      SMBvariables[SMBV_BattStatus][hibyte] |= TERMINATE_DISCHARGE_ALARM;
    }
    return;		//don't bother running any further tests!
  }
  else
  {
    FCSR |= (1<<DFE) | (1<<CFE) | (1<<PFD);
  }

  //Next, check the Charging or Discharging current levels for excesses.
  charge_percent = RelativeSOC();
  cell = Current1Sec();		//neg means discharging, pos means charging
  if(cell <= 0)			//discharging (0 is included due to SELF-discharge)
  {
    cell = -cell;

    //We must now assert the DISCHARGING flag.
    SMBvariables[SMBV_BattStatus][lobyte] |= DISCHARGING;

    //If we're discharging, always clear the Charging alarm flag.
    SMBvariables[SMBV_BattStatus][hibyte] &= ~TERMINATE_CHARGE_ALARM;

    //See if we should clear the FULLY_CHARGED flag yet.
    if(charge_percent < 99)
      SMBvariables[SMBV_BattStatus][lobyte] &= ~FULLY_CHARGED;

    //Check if need to Terminate Discharge (bit D11)	
    if((cell > PACK_DISCHG_MAX) || (charge_percent == 1))
      SMBvariables[SMBV_BattStatus][hibyte] |= TERMINATE_DISCHARGE_ALARM;
    else
      SMBvariables[SMBV_BattStatus][hibyte] &= ~TERMINATE_DISCHARGE_ALARM;

    //Note: we ONLY shut off LOCALLY if the cell V drops below
    // the minimum CELL VOLTAGE, and NOT if (charge_percent == 0).
    //The reason for this is, by definition, the pack is not drained
    // completely until you get down to minimum V. The charge_percent
    // figure could be *wrong* or it may be uncalibrated.
  }
  else //we are Charging
  {
    //When charging, always clear the Discharging alarm flag.
    SMBvariables[SMBV_BattStatus][hibyte] &= ~TERMINATE_DISCHARGE_ALARM;

    //We can therefore also clear the DISCHARGING flag.
    SMBvariables[SMBV_BattStatus][lobyte] &= ~DISCHARGING;

    //See if we can clear the FULLY_DISCHARGED flag yet.
    if( (SMBvariables[SMBV_BattStatus][lobyte] & FULLY_DISCHARGED) &&
        (charge_percent > 20) )
      SMBvariables[SMBV_BattStatus][lobyte] &= ~FULLY_DISCHARGED;

    if(cell == 0)	//is there NO charge current?
    {		// currently never reached, due to 0 included in discharge
      SMBvariables[SMBV_BattStatus][hibyte] &= ~TERMINATE_CHARGE_ALARM;
      SMBvariables[SMBV_BattStatus][hibyte] &= ~OVER_CHARGED_ALARM;
    }
    else //still charging...
    {
      //Terminate Charge (bit D14)
      if((cell > PACK_MAX_CHG_C) || (charge_percent >= 100))
        SMBvariables[SMBV_BattStatus][hibyte] |= TERMINATE_CHARGE_ALARM;
      else
        SMBvariables[SMBV_BattStatus][hibyte] &= ~TERMINATE_CHARGE_ALARM;

      //Excess charge?
      if(charge_percent >= 100)
        SMBvariables[SMBV_BattStatus][hibyte] |= OVER_CHARGED_ALARM;
      else
        SMBvariables[SMBV_BattStatus][hibyte] &= ~OVER_CHARGED_ALARM;
    }
  }

  //! \todo Whenever you move 2X the capacity, either into or out of the pack, you are supposed
  //! to increment the CycleCount variable. You do NOT have to have hit both FULL and EMPTY limits.
  //
  // if(???)
  //   SMBvar_int[SMBV_CycleCount]++;
}



void ThermalCheck(void)
{
  unsigned int temp = GetTemperature();

  //check for OverTemp alarm (bit D12)
  if(temp > MAX_CELL_TEMPERATURE)
    SMBvariables[SMBV_BattStatus][hibyte] |= OVER_TEMP_ALARM;
  else
    SMBvariables[SMBV_BattStatus][hibyte] &= ~OVER_TEMP_ALARM;

  //If the Host/Charger doesn't respond fast enough, SHUT DOWN LOCALLY.
  if(temp > MAX_ONCHIP_TEMP)
    DoShutdown(SHUTDOWN_REASON_OVERTEMPERATURE);


//! \todo  The following Cell temperature checking is disabled since it
//!        is left to the user to choose a specific Thermistor.

/*
  if(ReadTemperature(1) > MAX_CELL_TEMPERATURE)
    ;


  if(ReadTemperature(2) > MAX_CELL_TEMPERATURE)
    ;

  if(PACKSTACK <= 2)
    return;
  if(ReadTemperature(3) > MAX_CELL_TEMPERATURE)
    ;


  if(PACKSTACK <= 3)
    return;
  if(ReadTemperature(4) > MAX_CELL_TEMPERATURE)
    ;
*/

}



//This makes sure Alarm Mode is only disabled for 60 seconds max.
void AlarmModeDisabledCheck(void)
{
  if(SMBvariables[SMBV_BattMode][hibyte] & ALARM_MODE)
  {
    if(++AlarmModeDisabledTimer >= 59)
    {
      SMBvariables[SMBV_BattMode][hibyte] &= ~ALARM_MODE;	//re-enable
    }
  }
}



void AlarmConditionsCheck(void)		//look for 'other' alarm conditions
{
  //Remaining Time (bit D8)
  if(SMBvar_int[SMBV_RemTimeAlm] != 0)
  {
    if(TimeToEmpty(0) < SMBvar_int[SMBV_RemTimeAlm])
      SMBvariables[SMBV_BattStatus][hibyte] |= REMAINING_TIME_ALARM;
  }
  else
    SMBvariables[SMBV_BattStatus][hibyte] &= ~REMAINING_TIME_ALARM;


  //Remaining Capacity (bit D9)
  if(SMBvar_int[SMBV_RemCapAlm] != 0)
  {
    if(RemainingCap() < SMBvar_int[SMBV_RemCapAlm])
      SMBvariables[SMBV_BattStatus][hibyte] |= REMAINING_CAPACITY_ALARM;
  }
  else
    SMBvariables[SMBV_BattStatus][hibyte] &= ~REMAINING_CAPACITY_ALARM;

}



void MasterSMBusCheck(unsigned char ResetTimer)	//this gets called once per second (except to assert ResetTimer).
{
  static unsigned char timer = 0;

  //When you call SMB_Master(), it will only TX if there's something in the buffer.
  //To control this, we only INSERT if it's >10 seconds since last TX.

  if(ResetTimer)
  {
    timer = 0;
    return;
  }

  if(++timer < 10)
  {
    SMB_Master();
    return;
  }

  if(!(SMBvariables[SMBV_BattMode][hibyte] & ALARM_MODE))
  {
    //Check if there's a newly-updated BatteryStatus that must be sent ASAP
    if(SMBvariables[SMBV_BattStatus][hibyte])
    {
      if(SMBvariables[SMBV_BattStatus][hibyte] & ~(REMAINING_CAPACITY_ALARM | REMAINING_TIME_ALARM))
      { // Need to send this message to BOTH the Host and the Charger.
        MasterInsertMsg(CHARGER_ADDR, SMB_M_CMD_ALARMWARNING, SMBvar_int[SMBV_BattStatus]);
      }
      //All messages are sent to the Host.
      MasterInsertMsg(HOST_ADDR, SMB_M_CMD_ALARMWARNING, SMBvar_int[SMBV_BattStatus]);
    }


    //! \todo  Only a very basic charge control algorithm has been implemented. It uses the charge state to decide.
    //!        It is up to the designer to implement a more sophisticated charging algorithm.

    if(!(SMBvariables[SMBV_BattMode][hibyte] & CHARGER_MODE))    // Check that Charger_Mode is enabled, per sbdat110, 5.1.4
    {
      if(!(SMBvariables[SMBV_BattStatus][lobyte] & FULLY_CHARGED) ||
         !(SMBvariables[SMBV_BattStatus][hibyte] & TERMINATE_CHARGE_ALARM) ||
         !(SMBvariables[SMBV_BattStatus][hibyte] & OVER_CHARGED_ALARM)) { // it is OK to charge?

        SMBvar_int[SMBV_ChgCurrent] = PACK_CCCV_C;	//
        SMBvar_int[SMBV_ChgVoltage] = PACK_CCCV_V;	//
        MasterInsertMsg(CHARGER_ADDR, SMB_M_CMD_CHARGINGCURRENT, PACK_CCCV_C);
        MasterInsertMsg(CHARGER_ADDR, SMB_M_CMD_CHARGINGVOLTAGE, PACK_CCCV_V);
      } else { // it is not ok to charge
        SMBvar_int[SMBV_ChgCurrent] = 0x0000;	//
        SMBvar_int[SMBV_ChgVoltage] = 0x0000;	//
      }
    }


    timer = 0;	//clearing this prevents the same messages from repeatedly being added to the TX buffer.
  }

  SMB_Master();
}





void SleepMgr(void)
{
  SMCR |= (1<<SE);
  __sleep();
  __no_operation();
  __no_operation();
  __no_operation();
  __no_operation();

  //at this point, we should be asleep.

  // ...

  //at this point, we've just completed running the ISR for whatever interrupt woke us up.
  SMCR &= ~(1<<SE);	//According to the datasheet, we should clear the SE bit at wakeup.
  return;		//Go back to main loop & see if we need to do anything else before sleeping again.
}



void WdogMgr(void)
{
  //! \todo  This code can be made more sophisticated to prevent runaways
  //        from continually feeding the Wdog.

  __watchdog_reset();

  return;
}



void DoCalibrate(void)
{
  // Explanation of the DoCalibrate routine:
  // User requests calibration by sending a word write SMBus command 0x3c (OptionalMfgFunction4)
  // with the appropriate bits set. This sets CALIBREQUESTED bit flag in action_flags, and the
  // 'calibration_state_req' variable contains the received word. A read of OptionalMfgFunction4
  // returns the current state contained in the 'calibration_state variable'.
  //
  // The valibles are both divided into 2-bit fields, which makes room for a total of 8 calibration routines.
  // Currently two calibration-routines are implemented in the lower 4 bits of the lowest byte:
  // xx xx xx xx xx xx BB AA.
  //
  // xx = Unused bits, always 00.
  // AA = State of vref calibration.
  // BB = State of coloumb counter offset calibration.
  //
  // State codes:
  //   00 - No calibration / calibration not requested.
  //   01 - Calibration on-going / calibration requested.
  //   10 - Calibration FAILED / request to delete calibration.
  //   11 - Calibration OK / revert to old calibration if calibration failed and valid values exist
  //                         otherwise return to No calibration
  //
  // See diagram in documentation for details on state transitions. A failed calibration resets the
  // values to Atmel factory defaults, but since the values are still in eeprom a reset will load them
  // and set calibration_state to calibration OK. A write with Calibration OK will correspondingly restore
  // valid calibration values and set state to Calibration OK, or to No calibration if not valid values.
  // Note: A write with calibration fail will delete previous calibration values.

  unsigned char state, request;
  unsigned int backup_state_req = calibration_state_req;

  state = ((calibration_state & CAL_VREF_MASK) >> CAL_VREF_BIT_POS);
  request = ((backup_state_req & CAL_VREF_MASK) >> CAL_VREF_BIT_POS);
  switch(request) {
  case RUN_CAL:                  // calibration requested
    __disable_interrupt();
    DisableCellBalancing();
    calibration_state &= ~CAL_VREF_MASK; // Clear bits.
    calibration_state |=  CAL_VREF_RUN;  // Set state to RUN.
    if(CalibrateVREF()) {   // Calibration ok?
      calibration_state &= ~CAL_VREF_MASK; // Clear bits.
      calibration_state |=  CAL_VREF_OK; // Set state to OK.
      __enable_interrupt();
    } else {
      calibration_state &= ~CAL_VREF_MASK; // Clear bits.
      calibration_state |=  CAL_VREF_FAIL; // Set state to FAIL.
      __enable_interrupt();
    }
    EnableCellBalancing();
    break;
  case CAL_FAIL:             // delete calibration
    if (state == NO_CAL) {   // not relevant in state NO_CAL
      break;
    } else {                 // delete calibration constants
      do {} while(EECR & (1<<EEPE));
      do {} while(SPMCSR & (1<<SPMEN));
      EEAR = EESTORAGE_BGCCR;
      EEDR = 0xFF;
      EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
      EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go
      calibration_state &= ~CAL_VREF_MASK; // Clear bits.
      ReadVrefCalibration();
   }
    break;
  case CAL_OK:                 // go to eeprom values if valid values exist
    if (state == CAL_FAIL) {   // only valid from CAL_FAIL state
      if (ReadVrefCalibration()) {
        calibration_state &= ~CAL_VREF_MASK; // Clear bits.
        calibration_state |=  CAL_VREF_OK;   // Set state to OK.
      } else {
        calibration_state &= ~CAL_VREF_MASK; // Clear bits.
      }
    }
    break;
  default:
    break;
  }


  state = ((calibration_state & CAL_CC_MASK) >> CAL_CC_BIT_POS);
  request = ((backup_state_req & CAL_CC_MASK) >> CAL_CC_BIT_POS);
  switch(request) {
  case RUN_CAL:                // calibration requested
    __disable_interrupt();
    calibration_state &= ~CAL_CC_MASK;  // Clear bits.
    calibration_state |=  CAL_CC_RUN;   // Set state to RUN.
    if(CalibrateCCoffset()) {   // Calibration ok?
      calibration_state &= ~CAL_CC_MASK; // Clear bits.
      calibration_state |=  CAL_CC_OK; // Set state to OK.
      __enable_interrupt();
    } else {
      calibration_state &= ~CAL_CC_MASK; // Clear bits.
      calibration_state |=  CAL_CC_FAIL; // Set state to FAIL.
      __enable_interrupt();
    }
    break;
  case CAL_FAIL:             // delete calibration
    if (state == NO_CAL) {   // not relevant in state NO_CAL
      break;
    } else {                 // delete calibration valid
      do {} while(EECR & (1<<EEPE));
      do {} while(SPMCSR & (1<<SPMEN));
      EEAR = EESTORAGE_CC_valid;
      EEDR = 0xFF;
      EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (0<<EEPE) | (0<<EERE);	//arm
      EECR = (0<<EEPM1) | (0<<EEPM0) | (0<<EERIE) | (1<<EEMPE) | (1<<EEPE) | (0<<EERE);	//go
      calibration_state &= ~CAL_CC_MASK;   // Clear bits.
      ReadCCOffsetCalibration();
    }
    break;
  case CAL_OK:                 // go to eeprom values if valid values exist
    if (state == CAL_FAIL) {   // only valid from CAL_FAIL state
      if (ReadCCOffsetCalibration()) {
        calibration_state &= ~CAL_CC_MASK; // Clear bits.
	    calibration_state |=  CAL_CC_OK;   // Set state to OK.
      } else {
        calibration_state &= ~CAL_CC_MASK; // Clear bits.
      }
    }
    break;
  default:
    break;
  }
}




/*  ASSORTED DOCUMENTATION

Automatic Cell Balancing
Once per second, a scan is done of all cell voltages.  However, if the cell-balancing
FETs are enabled at this time, they will disrupt the voltage measurements.  To handle
this, the VADC ISR will shut off the balancing FETs during the scan (early enough for
the input RC filters to stabilize to less than 1mV).  At the completion of the cell
voltage scan, if the cells were being balanced the correct balancing FET will be
re-enabled automatically.

Impedance Measurement
In order to measure a cell's impedance, the cell's voltage must be measured concurrently
with the instantaneous current.  Therefore, a copy of the instantaneous current reading
corresponding to the time when each cell's voltage conversion occurs is stored in the
array cell_current[4].  A mainline routine can then manage this data as appropriate.
Four individual copies are saved in case a CCADC instantaneous interrupt occurs during
a cell voltage reading scan.

*/


/*! \mainpage
* This documents functions, variables, typedefs, enumerations, enumerator and defines
* in the the software for ATAVRSB100.\n
* Please see the Related Pages for Compilation Info, Release Notes and a Todo List
*/

/*! \page doc_page1 Compilation Info
* \section section Main source
* This software was written for the IAR Embedded Workbench, 4.11A/4.10B, but can also be
* built using 3.20C (see note futher down).\n
* To make project:
* Add the .c files to project (main.c, gpio.c, main.c, pwrmgmt.c, safety.c, smbus.c
* and timer.c, but not the bootloader bootloader_smbus.c),
* use device --cpu=m406, CLIB Heap size 0x80, Data stack (CSTACK) 0x80 bytes,
* Return adress stack (RSTACK) 64 levels, enable bit definitions in I/O include files,
* optimization low, output format: ubrof8 for Debug and intel_extended for
* Release \n
* If using IAR EWAVR 3.20, use Prosessor configuration -v3 and override default linker
* command file with: $PROJ_DIR$\\cfgm406s.xcl (file included with the source). You also
* need to edit analog.c (see todo in CC_Accumulator_ISR(void)).
*/

/*! \page doc_page2 Release Notes
*  Note that this source code is preliminary. This release targets ATmega406 rev E,
*  older revisions of ATmega406 are not supported.
*  The following features are not fully implemented or are not completely verified:\n
*  - Temperature monitoring of battery cells requires additional code.\n
*  - Encryption may be added to bootloader updates but is not presently implemented.\n
*/


