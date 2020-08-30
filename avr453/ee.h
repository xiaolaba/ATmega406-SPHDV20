/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Allocation of EEPROM storage.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/ee.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


// Byte 0 of the E2 is not used, as a precaution against accidental erasure.
// After each read or write operation, the EEADR register is reset to zero,
// ensuring that an accidental erase or write will not destroy valid data.


// Defines the allocation of EEPROM storage for calibration information.

#define EESTORAGE_SHUTDOWNREASON 0x01

#define CELL1_25C 0x10
#define CELL2_25C 0x12
#define CELL3_25C 0x14
#define CELL4_25C 0x16

#define EESTORAGE_BGCRR      0x02
#define EESTORAGE_BGCCR      0x03
#define EESTORAGE_CC_valid   0x04
#define EESTORAGE_CCIoffset  0x05
#define EESTORAGE_CCoffset   0x06
