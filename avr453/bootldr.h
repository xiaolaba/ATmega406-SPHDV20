/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Headerfile for bootloader_smbus.c
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
 * $Name$
 * $Revision: 2687 $
 * $RCSfile$
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


//Prototypes...

#define SMBvariables SV.SMBvar
#define SMBvar_int   SV.SMBint




#define HOST_ADDR    0x10
#define CHARGER_ADDR 0x12
#define BATTERY_ADDR 0x16


// Note that the following cmds are both Master Write to Charger, and Slave Read.
#define SMB_M_CMD_CHARGINGCURRENT 0x14
#define SMB_M_CMD_CHARGINGVOLTAGE 0x15

//This command is Master Write only, to both the Charger & Host addresses
#define SMB_M_CMD_ALARMWARNING 0x16


//SMBus Variable + Command Name offset list
#define SMBV_MfrAccess  0
#define SMBV_RemCapAlm  1
#define SMBV_RemTimeAlm 2
#define SMBV_BattMode   3
#define SMBV_AtRate     4
#define SMBV_AtRateTTF  5
#define SMBV_AtRateTTE  6
#define SMBV_AtRateOK   7

#define SMBV_Temperature 8
#define SMBV_Voltage     9
#define SMBV_Current    10
#define SMBV_AvgCurrent 11
#define SMBV_MaxError   12
#define SMBV_RelSOC     13
#define SMBV_AbsSOC     14
#define SMBV_RemCap     15

#define SMBV_FullChgCap 16
#define SMBV_RunTTE     17
#define SMBV_AvgTTE     18
#define SMBV_AvgTTF     19
#define SMBV_ChgCurrent 20
#define SMBV_ChgVoltage 21
#define SMBV_BattStatus 22
#define SMBV_CycleCount 23

#define SMBV_DesignCap  24
#define SMBV_DesignVolt 25
#define SMBV_SpecInfo   26
#define SMBV_MfrDate    27
#define SMBV_SerialNo   28

#define SMBV_MfrName    32
#define SMBV_DeviceName 33
#define SMBV_DeviceChem 34
#define SMBV_MfrData    35
#define SMBV_Opt5     0x2F
#define SMBV_Opt4     0x3C
#define SMBV_Opt3     0x3D
#define SMBV_Opt2     0x3E
#define SMBV_Opt1     0x3F



//Offsets within each of the 29 SMBvariables elements
#define lobyte 0
#define hibyte 1


//Offsets within TW_RxBuf[]
#define TWRX_BLKCNT  2  /* amount of remaining bytes in this transmission */
#define TWRX_CMD     3  /* flag for the specific cmd to be done */
#define TWRX_MEM     4  /* flag indicating which memory space */
#define TWRX_OFFSET  5  /* offset into the SRAM buffer for this data */
#define TWRX_LOADDR  6  /* hi byte of mem space address    */
#define TWRX_HIADDR  7  /* hi byte of mem space address    */
#define TWRX_SIZE    8  /* size of data block that follows */
#define TWRX_DATA    9  /* start of data block, if present */



//SMBV_BatteryMode bit definitions

//  HiByte
#define CHARGE_CONTROLLER_ENABLED	0x01
#define PRIMARY_BATTERY			0x02
#define ALARM_MODE			0x20
#define CHARGER_MODE			0x40
#define CAPACITY_MODE			0x80

//  LoByte
#define INTERNAL_CHARGE_CONTROLLER	0x01
#define PRIMARY_BATTERY_SUPPORT		0x02
#define CONDITION_FLAG			0x80



//SMBV_BatteryStatus bit definitions.

// HiByte
/* * * * * * Alarm Bits * * * * */
#define OVER_CHARGED_ALARM		0x80
#define TERMINATE_CHARGE_ALARM		0x40
#define OVER_TEMP_ALARM			0x10
#define TERMINATE_DISCHARGE_ALARM	0x08
#define REMAINING_CAPACITY_ALARM	0x02
#define REMAINING_TIME_ALARM		0x01

// LoByte
/* * * * * * Status Bits * * * * */
#define INITIALIZED		0x80
#define DISCHARGING		0x40
#define FULLY_CHARGED		0x20
#define FULLY_DISCHARGED	0x10

/* * * * * * Error Codes * * * * */
#define SMBerr_OK               0x00	// r/w The Smart Battery processed the function code without detecting any errors.
#define SMBerr_Busy             0x01	// r/w The Smart Battery is unable to process the function code at this time.
#define SMBerr_ReservedCommand  0x02	// r/w The Smart Battery detected an attempt to read or write to a function code
					//     reserved by this version of the specification. The Smart Battery detected
					//     an attempt to access an unsupported optional manufacturer function code.
#define SMBerr_UnsuptdCommand   0x03	// r/w The Smart Battery does not support this function code which is
					//     defined in this version of the specification.
#define SMBerr_AccessDenied     0x04	//  w  The Smart Battery detected an attempt to write to a read-only function code.
#define SMBerr_OverUnderflow    0x05	// r/w The Smart Battery detected a data overflow or under flow.
#define SMBerr_BadSize          0x06 	//  w  The Smart Battery detected an attempt to write to a function code with
					//     an incorrect size data block.
#define SMBerr_UnknownError     0x07 	// r/w The Smart Battery detected an unidentifiable error.

