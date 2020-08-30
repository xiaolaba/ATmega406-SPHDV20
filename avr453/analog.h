/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for analog.c.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/analog.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//Prototypes

void SetMaxTopAcc(long value);

void CalculateADCresults(void);

void CCinit(void);				//call this at startup
enum CC_MODE {CC_DISABLED = 0, CC_ACCUMULATE, CC_REGULAR};
void CCmode(unsigned char mode);		//call this to switch modes


void ADCinit(void);				//call this at startup ASAP to cal the osc's.
void StartAdc(unsigned char select);		//starts a scan of all ADC channels.
unsigned int ReadTemperature(unsigned char channel);	//1-4 = thermistors

void FullChargeReached(void);			//call this when detect ChargingComplete condition
void FullDischargeReached(void);		//call this when detect DischargeComplete condition
unsigned int GetVoltage(void);			//also serves as "cmd = 9"
unsigned int ReadCell(char);			//Returns individual cell V in mV (cell = 1-4)

void DisableCellBalancing(void);
void EnableCellBalancing(void);		//if CellToBalance = 0, no balancing req'd.

unsigned char ReadVrefCalibration(void);   // read VREF calibration constants from eeprom or use factory defaults
unsigned char ReadCCOffsetCalibration(void); // read C offset constants from eeprom or use 0


// Analog calculations to support specific SMBus Slave Read commands
unsigned int AtRateTTF(void);			// cmd = 5
unsigned int AtRateTTE(void);			// cmd = 6
unsigned int AtRateOK(void);			// cmd = 7
unsigned int GetTemperature(void);		// cmd = 8
signed int Current1Sec(void);			// cmd = 10  // positive value means Charging, negative means Discharging.
signed int CCarray_Average(void);		// cmd = 11  // positive value means Charging, negative means Discharging.
unsigned char RelativeSOC(void);		// cmd = 13
unsigned int AbsoluteSOC(void);			// cmd = 14
unsigned int RemainingCap(void);		// cmd = 15
unsigned int FullChgCap(void);			// cmd = 16
unsigned int TimeToEmpty(unsigned char avgd);	// cmd = 17,18
unsigned int AvgTimeToFull(void);		// cmd = 19


//Variables

#ifdef MODULE_ANALOG

  volatile signed int LatestCCI = 0;   // most recent value of Instantaneous CC conversion
  signed long CCoffset = 0x0000;		   // this is SUBTRACTED to EACH sample from the CC Accum.
  signed int CCIoffset = 0;            // this is in mA and is ONLY SUBTRACTED to the 1-second total, NOT TO EACH SAMPLE.

  unsigned char FastRCCal = 0;
  unsigned int SlowRCCal = 0;
  unsigned char BGCCRCal = 0;
  unsigned char ThermistorSelect = 0;
  unsigned char CellToBalance = 0;
  unsigned int cell_current[4];	        //used for impedance measurement
  signed long RunningAcc = 0;

#else

  extern volatile signed int LatestCCI;
  extern unsigned char FastRCCal;
  extern unsigned int SlowRCCal;
  extern unsigned char BGCCRCal;
  extern unsigned char ThermistorSelect;
  extern unsigned char CellToBalance;
  extern unsigned int cell_current[4];
  extern signed long RunningAcc;

#endif



//Defines

#define ACCUM_CONV_TIME_125  ((0<<CADAS1) | (0<<CADAS0))
#define ACCUM_CONV_TIME_250  ((0<<CADAS1) | (1<<CADAS0))
#define ACCUM_CONV_TIME_500  ((1<<CADAS1) | (0<<CADAS0))
#define ACCUM_CONV_TIME_1000 ((1<<CADAS1) | (1<<CADAS0))

#define ACCUM_CONV_TIME ACCUM_CONV_TIME_1000


#define REG_CONV_TIME_250  ((0<<CADSI1) | (0<<CADSI0))
#define REG_CONV_TIME_500  ((0<<CADSI1) | (1<<CADSI0))
#define REG_CONV_TIME_1000 ((1<<CADSI1) | (0<<CADSI0))
#define REG_CONV_TIME_2000 ((1<<CADSI1) | (1<<CADSI0))

#define REG_CONV_TIME REG_CONV_TIME_1000

//The following is the amount that the Instantaneous reading must
// be shifted up to correspond to an Accumulator reading.  It is
// dependent on the REG_CONV_TIME parameter.  The delta if a one-second
// rate is used is a factor of 16.
#define REGULAR_CURRENT_SCALE_VALUE 4


//Since this is detected using the Instantaneous conversion,
// each count is 10.7mA if a 5mOhm sense resistor is used.
// A value of 9 corresponds to just under 100mA.
#define ACTIVE_CURRENT_THRESHOLD 9


//The following is to handle rev.E errata concerning VPTAT reading
#define VPTAT_READINGS 10   // to ensure correct VPTAT readings
#define ADC0_READINGS  10   // to ensure correct readings after VPTAT readings, if the
  							// ADC0-3 values are not used we could decrease this to 6.



