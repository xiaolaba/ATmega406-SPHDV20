/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Defines and variables for calibration.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/calibration.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/

// Prototypes
unsigned char CalibrateVREF(void);       // calibration routine for VREF
unsigned char CalibrateCCoffset(void);   // calibration routine for CC offset


#ifdef MODULE_CALIBRATION
unsigned int calibration_state = 0;
unsigned int calibration_state_req = 0;
#define CCoffset_limit  100            // limit for CCoffset (to be decided)
#define CCIoffset_limit 66             // limit for CCIoffset 10.5*(CCoffset/16)

// This section is for calibration on CELL1 with 4096mV
#define CAL_CHANNEL 		0x01		// Mux value for channel PV1
#define CAL_GAIN            ADCgain[0]  // Gain value for channel PV1
#define CAL_WAIT            10          // Number of VADC cycles (512us) to wait for error from
  								        // CellBalancing to cancel.

// Defines for the different temperature dependency calibration settings (thermometer coded)
#define TempCal0			0x00
#define TempCal1			0x01
#define TempCal2			0x03
#define TempCal3			0x07
#define TempCal4			0x0F
#define TempCal5			0x1F
#define TempCal6			0x3F
#define TempCal7			0x7F
#define TempCal8			0xFF

//! /todo The Vcalibration value have to be tuned to correct input voltage.
#define Vcalibration_value	67117056    // input voltage = 4096.5mV*16384/mV = 67117056
//#define Vcalibration_value	67094442        // debug input voltage = 4096.5mV*60BC/mV*1.10055/1.100
#define VLevel0             0				// padding to enable Tempcal0 if < VLevel1
#define VLevel1				-615753			// Preliminary
#define VLevel2				-491700
#define VLevel3				-337272
#define VLevel4				-152886
#define VLevel5				0
#define VLevel6				60960
#define VLevel7				121810
#define VLevel8				182549

// This section is for calibration on ADC0 with 0.55V, should be revised when algorithm is decided
/*
#define CAL_CHANNEL 		0x07		// Mux value for channel ADC0
#define CAL_GAIN            1           // Added to give compatibility with scaled inputs probably optimized away
  										// by compiler. Routine could be optimized if unscaled input is used.
#define CAL_WAIT            1           // Number of VADC cycles (512us) to wait for error from CellBalancing
  								        // to cancel. We always Waits 1 cycle for dummy read.

// Defines for the different temperature dependency calibration settings (thermometer coded)
#define TempCal0			0x00
#define TempCal1			0x01
#define TempCal2			0x03
#define TempCal3			0x07
#define TempCal4			0x0F
#define TempCal5			0x1F
#define TempCal6			0x3F
#define TempCal7			0x7F
#define TempCal8			0xFF

#define Vcalibration_value	2048				// Used with input voltage = 0.55V (half range)
#define VLevel1				-18			// Used for demonstration, values are TBD
#define VLevel2				-15
#define VLevel3				-10
#define VLevel4				-5
#define VLevel5				0
#define VLevel6				2
#define VLevel7				4
#define VLevel8				6
*/

__flash unsigned char tempcal[] = {TempCal0,TempCal1,TempCal2,TempCal3,TempCal4,TempCal5,TempCal6,TempCal7,TempCal8};
__flash signed long int vcalibration_level[] = {VLevel0,VLevel1,VLevel2,VLevel3,VLevel4,VLevel5,VLevel6,VLevel7,VLevel8};

#else
  extern unsigned int calibration_state;
  extern unsigned int calibration_state_req;
#endif


// Defines for calibration variables.

#define CAL_VREF_BIT_POS 0
#define CAL_CC_BIT_POS   2

#define NO_CAL     0
#define RUN_CAL    1
#define CAL_FAIL   2
#define CAL_OK     3

#define CAL_VREF_MASK  (3        << CAL_VREF_BIT_POS)
#define CAL_VREF_RUN   (RUN_CAL  << CAL_VREF_BIT_POS)
#define CAL_VREF_FAIL  (CAL_FAIL << CAL_VREF_BIT_POS)
#define CAL_VREF_OK    (CAL_OK   << CAL_VREF_BIT_POS)

#define CAL_CC_MASK    (3        << CAL_CC_BIT_POS)
#define CAL_CC_RUN     (RUN_CAL  << CAL_CC_BIT_POS)
#define CAL_CC_FAIL    (CAL_FAIL << CAL_CC_BIT_POS)
#define CAL_CC_OK      (CAL_OK   << CAL_CC_BIT_POS)
