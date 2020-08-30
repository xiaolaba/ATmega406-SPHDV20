/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for main.c
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/main.h $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


#define OP_MODE_FULL 0
#define OP_MODE_STDBY 1
#define OP_MODE_SLEEP 2

#ifdef MODULE_MAIN
  unsigned char action_flags = 0;
  unsigned char OperatingMode = OP_MODE_FULL;
#else
  extern unsigned char action_flags;
  extern unsigned char OperatingMode;
#endif


#define QTRSECFLAG     1
#define ADCDONEFLAG    2
#define ALARMMODEFLAG  4
#define MASTERSMBDONE  8
#define CALIBREQUESTED 16


#define CheckADCScanDone  (action_flags & ADCDONEFLAG)
#define SetADCScanDone    {action_flags |= ADCDONEFLAG;}
#define ClrADCScanDone    {action_flags &= ~ADCDONEFLAG;}

#define CheckQtrSec       (action_flags &  QTRSECFLAG)
#define SetQtrSec         {action_flags |= QTRSECFLAG;}
#define ClrQtrSec         {action_flags &= ~QTRSECFLAG;}

#define CheckAlarmMode    (action_flags &  ALARMMODEFLAG)
#define SetAlarmMode      {action_flags |= ALARMMODEFLAG;}
#define ClrAlarmMode      {action_flags &= ~ALARMMODEFLAG;}

#define CheckMasterSMBdone (action_flags & MASTERSMBDONE)
#define SetMasterSMBdone   {action_flags |= MASTERSMBDONE;}
#define ClrMasterSMBdone   {action_flags &= ~MASTERSMBDONE;}

#define CheckCalibRequest (action_flags & CALIBREQUESTED)
#define SetCalibRequest   {action_flags |= CALIBREQUESTED;}
#define ClrCalibRequest   {action_flags &= ~CALIBREQUESTED;}

