/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Battery pack configuration.
 *
 *       This file defines the number of cells in series, as well as in
 *       parallel. Modify the values to match your pack's configuration.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/pack_cfg.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//! \todo  Modify these settings to match the battery configuration in your design.

#define PACKWIDTH 3	/* # of cells used in parallel to increase the pack's current capability. */
#define PACKSTACK 4	/* # of cells stacked in series; must be ONLY   2, 3 or 4   for the m406! */

