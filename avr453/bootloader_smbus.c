/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       SMBus Bootloader for ATmega406
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


/*! \page doc_page1 Compilation Info
* \section Bootloader
* This software was written for the IAR Embedded Workbench, 4.11A/4.10B, but can also be
* built using 3.20C \n
* To make project:
* Add this file, bootloader_smbus.c to project, use device -v3, enhanced core,
* no RAMPZ register, enable bit definitions in I/O include files, optimization was set
* to medium, override default linker command file with: $PROJ_DIR$\\m406s_2kwBootLdr.xcl
* (included with the source), output format: ubrof8 for Debug and intel_extended for Release.\n
* To program the device with bootloader, one first has to erase the device and program the
* bootloader, then program the main section without erasing the device first.
*/


//#define ENABLE_TESTCODE


//#include <pgmspace.h>
#include <inavr.h>

#define MODULE_BOOTLDR		/* ensure that we instantiate our variables in bootldr.h */
#include "bootldr.h"
//#include "iom406_320.h"
#include "iom406.h"     // IAR headerfile for Mega406 (EW 410)
#include "flash.h"


void TWI_handler(void);
void SMB_RestoreBus(void);
unsigned char FastCRC(unsigned char LastCRC, unsigned char newbyte);
void SMB_BusTimeout(void);
void SMB_Reply(void);
void SMB_CmdInterpreter(void);
void LoopMemory(void);


//=======================================================================

//This is used to check for an out-of-bounds SMBus command.
#define HIGHEST_SMB_CMD 0x3F


//Two-Wire-Interface TWSR (STATUS) values
//Note that since our Prescale value is 0, we don't need to MASK the Status byte.

//  Globally applicable TWI status codes:
#define TWS_MASK  0xF8		/* Two-Wire Status Mask */
#define TWS_NSTAT 0xF8		/* No Status Available now */

//  MASTER-related Status codes:
#define TWS_START 0x08
#define TWS_RESTART 0x10
#define TWS_WRITE_ACK 0x18	/* sent a SLA+W, got ACK */
#define TWS_WRITE_NAK 0x20	/* sent SLA+W, got NAK */
#define TWS_TXDATA_ACK 0x28	/* Data tx'd & was ACK'd */
#define TWS_TXDATA_NAK 0x30	/* Data tx'd & was NAK'd */
#define TWS_LOST_ARB 0x38	/* lost bus arbitration */

#define TWS_READ_ACK 0x40	/* got ACK from a SLA+R request */
#define TWS_READ_NAK 0x48	/* got NAK from a SLA+R request */
#define TWS_RXDATA_ACK 0x50	/* We rcvd data and ACKd back */
#define TWS_RXDATA_NACK 0x58	/* We rcvd data and we NACKd back */


//  SLAVE-related Status codes:
#define TWS_SLA_W 0x60		/* Got SLA + Write */
#define TWS_SLA_R 0xA8		/* Got SLA + Read  */
#define TWS_RDATA 0x80		/* Got a data byte */
#define TWS_RCMD  0x80		/* Got a command byte */
#define TWS_RSTOP 0xA0		/* Got a Stop */
#define TWS_REPEAT 0xA0		/* Got a Repeated-Start */
#define TWS_RACK  0xB8		/* Send a data byte and got an ACK back */
#define TWS_RNAK  0xC0		/* Sent a data byte and got a NAK back */
#define TWS_FINAL 0xC8		/* Sent the final byte, got ACK back */
#define TWS_BERR  0x00		/* Saw a Bus Error */


// Two-Wire CONTROL values

#define TWC_GO 0x85	    /* clr TWINT; assert ENA & IntEna */
#define TWC_READ_NoACK 0x85 /* read a byte, but don't ACK when done */
#define TWC_START 0xA5	    /* send START bit, assert ENA & IntEna */
#define TWC_STOP 0x94	    /* leave INT *DISabled* when done */
#define TWC_RESTART 0xB5    /* send STOP, then START again; INT ena */


//=======================================================================


unsigned char TW_TxBuf[36];	//must be long enough for any outbound strings
unsigned char TW_TxBufCnt;	//how many valid bytes are in the buffer
unsigned char TW_TxBufIndex;

unsigned char TW_RxBuf[36];	//must be big enough for inbound programming data
unsigned char TW_RxBufCnt;	// SLA + Cmd + Count + 32bytes + PEC = 36
unsigned char TW_RxBufIndex;

unsigned char TW_state;		//state variable
unsigned char UsePEC;		//PEC usage is disabled by default.

unsigned char LoopFlag;		//if we're looping on a slow memory operation
unsigned char BigData;		//flag, if doing multi=packet 'I' command
unsigned char src_i;
unsigned char dest_i;
unsigned char ctr;
unsigned char lomemptr;
unsigned char himemptr;
unsigned char __flash *fptr;
unsigned int eptr;

unsigned char SRAMbuffer[128];          //this is the size of a Flash page (64 *WORDS*)

//This byte contains flags from the TWI handler to tell the Foreground code what to do.
//If this byte is ever non-zero, the foreground code will act on its contents.
//Although it is written by both the Handler and the Foreground code, it does not
//  need to be declared VOLATILE because the SMBus is halted until the foreground
//  code finishes processing the associated command and has cleared this flag byte.
unsigned char TWI_CmdFlags;
  #define SMB_GenBusTimeout 1	/* Tell Foreground to generate a bus timeout, as we saw an error! */
  #define SMB_SetUpReply 2	/* Have Foreground set up TW_TxBuf[]. */
  #define SMB_GotCmdData 4	/* Have Foreground interpret the complete received command. */


unsigned char Status;           //this is a global variable and is the sole response to a READ using cmd 0x2F.
  #define SUCCESS 0
  #define BUSY    1
  #define BADPARAM 2
  #define CRCERROR 3
  #define FAILURE  0xFF


//=======================================================================

// Reset Management theory:
//
// To make it easier to enter the bootloader when the OptionalMfgCmd5 command is received
// while running the SMBus interpreter in the application code, we monitor the Reset
// Source flags.  If we arrive here without any flags being asserted, then we can assume
// that we have jumped here from the application, and we can therefore assume that it is
// intended that the bootloader code be run.
// Otherwise, if any reset flag IS asserted when we enter, we check if there is a valid
// vector present at 0x0000; if not, we run the bootloader in order to retrieve a valid
// image, otherwise, we jump down to the application program and run it.
//
//! \todo  EXCEPTION: if the Watchdog flag is asserted, there is a POSSIBILITY that there's
//! a problem with the App code image.  It is left to the user to determine if this is the
//! case and to take remedial action.

void __low_level_init(void)
{
  unsigned char __flash * ptr = 0;

  if(!(MCUSR))    //If no reset source is asserted, we probably came here from the App code deliberately.
    return;       //Assume we need to run the BootLoader.

  // Some reset source was asserted. If it was Wdog, there may be a code problem and we
  // may need to handle it differently from other reset sources.  If so, add that code here.
  if(MCUSR & (1<<WDRF))
  {
    //! \todo  Add code here to handle Wdog reset.
  }

  //Some reset source was asserted, so if Reset vector is FFFF, assume we need to run the bootloader,
  // otherwise jump to the application code reset vector at 0x0000 and assume the App code is OK.
  if((*ptr++ != 0xFF) || (*ptr != 0xFF))
  {
    MCUCR &= (1<<IVSEL);
    asm("jmp 0");
  }

  //At this point, we apparently need to run the Bootloader code, as App section image appears invalid.
}




void init_boot(void)
{
  __disable_interrupt();
  TW_TxBufCnt = 0;	//how many valid bytes are in the buffer
  TW_TxBufIndex = 0;
  TW_RxBufCnt = 0;
  TW_RxBufIndex = 0;
  TW_state = 0;
  UsePEC = 0;
  Status = SUCCESS;
  LoopFlag = 0;
  BigData = 0;

  SMB_RestoreBus();
  TWBCSR = (1<<TWBCIF) | (1<<TWBDT1) | (1<<TWBDT0) | (0<<TWBCIP);
}


/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */

//The following functions are included to demonstrate how the incoming
// data should be set up for SMBus-based ISP commands.

#ifdef ENABLE_TESTCODE

void BOOT_TEST_EF(void)
{
  //Test the Flash Page Erase functionality (put junk at 0x0000-0x007F first)
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'E';     //Erase
  TW_RxBuf[TWRX_MEM] = 'F';	//flash
  TW_RxBuf[TWRX_LOADDR] = 0x80;
  TW_RxBuf[TWRX_HIADDR] = 0;
  UsePEC = 0;
}

void BOOT_TEST_EE(void)
{
  //Test the EEPROM Erase functionality (put junk at 0x1F0-0x1FF first; should only erase through 0x1F9)
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'E';     //Erase
  TW_RxBuf[TWRX_MEM] = 'E';	//EEPROM
  TW_RxBuf[TWRX_LOADDR] = 0xF0;
  TW_RxBuf[TWRX_HIADDR] = 0x01;
  TW_RxBuf[TWRX_SIZE] = 0x0a;	//only do 10 bytes (test non-power-of-2 case)
  UsePEC = 0;
}

void BOOT_TEST_I(void)
{
  unsigned char i;
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'I';     //Insert
  TW_RxBuf[TWRX_LOADDR] = 0;
  TW_RxBuf[TWRX_HIADDR] = 0;
  for(i=0; i<16; i++)
    TW_RxBuf[TWRX_DATA+i]=i+33;	//arbitrary data
  TW_RxBuf[TWRX_OFFSET] = 0x33;	//insert data into SRAM buffer starting at arbitrary offset 0x33
  TW_RxBuf[TWRX_SIZE] = 16;
  UsePEC = 0;
}

void BOOT_TEST_WF(void)
{
  unsigned char i;
  //Test the Flash Page WRITE functionality (store SRAMbuffer to flash 0x0000-0x007F)
  //Fill the buffer before doing WRITE tests.
  for(i=0; i<128; i++)
    SRAMbuffer[i]=i+33;
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'W';     //Write
  TW_RxBuf[TWRX_MEM] = 'F';	//flash
  TW_RxBuf[TWRX_LOADDR] = 0x80;
  TW_RxBuf[TWRX_HIADDR] = 0;
  UsePEC = 0;
}

void BOOT_TEST_WE(void)
{  //Test the EEPROM WRITE functionality (store small piece of SRAMbuffer to eeprom 0x1F0-0x1FB)
  unsigned char i;
  //Fill the buffer before doing WRITE tests.
  for(i=0; i<128; i++)
    SRAMbuffer[i]=i+33;
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'W';     //Write
  TW_RxBuf[TWRX_MEM] = 'E';	//EEPROM
  TW_RxBuf[TWRX_LOADDR] = 0xF0;	//any address is fine...
  TW_RxBuf[TWRX_HIADDR] = 0x01;
  TW_RxBuf[TWRX_OFFSET] = 0x33;	//use an arbitrary block from inside the buffer area
  TW_RxBuf[TWRX_SIZE] = 0x0c;
  UsePEC = 0;
}

void BOOT_TEST_PF(void)
{
  unsigned char i;
  //Verify that we can read back FLASH contents via the PATCH command.
  //First, clear SRAMbuffer.
  for(i=0; i<128; i++)
    SRAMbuffer[i] = 0;
  //Next, set up the PATCH command
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'P';     //Patch
  TW_RxBuf[TWRX_MEM] = 'F';	//flash
  TW_RxBuf[TWRX_LOADDR] = 0x80;	//any address is fine...
  TW_RxBuf[TWRX_HIADDR] = 0x00;
  UsePEC = 0;
}

void BOOT_TEST_PE(void)
{
  unsigned char i;
  //Verify that we can read back EEPROM contents via the PATCH command.
  //First, clear SRAMbuffer.
  for(i=0; i<128; i++)
    SRAMbuffer[i] = 0;
  //Next, set up the PATCH command
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'P';     //Patch
  TW_RxBuf[TWRX_MEM] = 'E';	//EEPROM
  TW_RxBuf[TWRX_LOADDR] = 0xE0;	//address+size must be less than 0x200...
  TW_RxBuf[TWRX_HIADDR] = 0x01;
  TW_RxBuf[TWRX_OFFSET] = 0x00;	//read it into the bottom of SRAMbuffer (arbitrary)
  TW_RxBuf[TWRX_SIZE] = 0x20;	//grab any size we want, up to 128 bytes
  UsePEC = 0;
}

unsigned char bigI;

void BOOT_TEST_bigI(void)
{
  unsigned char i;
  TW_RxBuf[TWRX_BLKCNT] = 32;
  TWI_CmdFlags = SMB_GotCmdData;
  TW_RxBuf[TWRX_CMD] = 'I';     //Insert
  for(i=0; i<26; i++)
    TW_RxBuf[TWRX_DATA+i]=i+1;	//arbitrary data
  bigI = i+1;                   //save for later
  TW_RxBuf[TWRX_OFFSET] = 0;	//insert data into SRAM buffer starting at arbitrary offset 0x33
  TW_RxBuf[TWRX_SIZE] = 128;
  UsePEC = 0;
}

void BOOT_TEST_bigI32(void)
{
  unsigned char i;
  TW_RxBuf[TWRX_BLKCNT] = 32;
  TWI_CmdFlags = SMB_GotCmdData;
  for(i=0; i<32; i++)
    TW_RxBuf[3+i]=i+bigI;	//arbitrary data
  bigI += i;                    //save for later
}

void BOOT_TEST_bigI8(void)
{
  unsigned char i;
  TW_RxBuf[TWRX_BLKCNT] = 6;
  TWI_CmdFlags = SMB_GotCmdData;
  for(i=0; i<6; i++)
    TW_RxBuf[3+i]=i+bigI;
}





void TestManager(void)
{
  static unsigned char testctr = 0;

  switch(testctr)
  {
    case 0:
      BOOT_TEST_I();
      testctr++;
      break;

    case 1:
      BOOT_TEST_WF();
      testctr++;
      break;

    case 2:
      BOOT_TEST_PF();
      testctr++;
      break;

    case 3:
      BOOT_TEST_EF();
      testctr++;
      break;

    case 4:
      if(!LoopFlag)
      {
        BOOT_TEST_WE();
        testctr++;
      }
      break;

    case 5:
      if(!LoopFlag)
      {
        BOOT_TEST_EE();
        testctr++;
      }
      break;

    case 6:
      if(!LoopFlag)
      {
        BOOT_TEST_PE();
        testctr++;
      }
      break;

    case 7: //test the chained Initialize command
      BOOT_TEST_bigI();
      testctr++;
      break;

    case 8:
      BOOT_TEST_bigI32();
      testctr++;
      break;

    case 9:
      BOOT_TEST_bigI32();
      testctr++;
      break;

    case 10:
      BOOT_TEST_bigI32();
      testctr++;
      break;

    case 11:
      BOOT_TEST_bigI8();
      testctr++;
      break;

    case 12:
      break;

  }
}

#endif

/* ************************************************************* */
/* ************************************************************* */
/* ************************************************************* */


void main(void)
{
  init_boot();

  for(;;)
  {

#ifdef ENABLE_TESTCODE
    TestManager();
#endif

    if(TWCR & (1<<TWINT))			//Note that the TWI handler is POLLED in the BootLoader.
      TWI_handler();

    if(TWI_CmdFlags)
    {
      if(TWI_CmdFlags == SMB_SetUpReply)	/* Have Foreground set up TW_TxBuf[]. */
      {
        TWI_CmdFlags = 0;
        SMB_Reply();
      }
      else
      if(TWI_CmdFlags == SMB_GotCmdData)
      {
        TWI_CmdFlags = 0;
        SMB_CmdInterpreter();
      }
      else
      if(TWI_CmdFlags == SMB_GenBusTimeout)	/* Tell Foreground to generate a bus timeout, as we saw an error! */
      {
        TWI_CmdFlags = 0;
        SMB_BusTimeout();
      }
    }

    if(LoopFlag)				/* handle a repeated SLOW memory operation off-line */
      LoopMemory();
  }
}










/* *************************************************************************
 *
 *   Low-Level SMBus Communications State Machine
 *
 ************************************************************************* */

//unsigned char TW_state = TW_IDLE;	//state variable

//State Machine states
enum /*TW_State*/ {TW_IDLE=0, TW_Wait4Cmd, TW_Wait4RW, TW_Wait4Data, TW_ReplyData, TW_MSLA_W, TW_MCMD_W, TW_MDATA_W };

void TWI_handler(void)
{
  unsigned char Status;

  Status = TWSR & 0xF8;		//This identifies what caused the interrupt to fire.

  switch(TW_state)
  {
    default:
    case TW_IDLE:	//If not SLA_W or RSTOP, is an error!
      if(TWS_SLA_W == Status)	// saw Slave address match with a Write bit
      {
        TW_state = TW_Wait4Cmd;
      }
      else
      if(TWS_RSTOP == Status)	//Saw a Stop, possibly left over from previous cmd.
      {
         ;			//Everything is probably OK.  Take no action.
      }
      else //had some type of error!
      {
        TWI_CmdFlags = SMB_GenBusTimeout;	//Flag the error & stay in this state.
        TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
        return;
      }
      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//must re-enable ACKing
      break;


    //SLAVE-mode states follow.

    case TW_Wait4Cmd:	//upon entry, we've expect to have received a Cmd byte.
      if(TWS_RCMD == Status)		//It appears that we have received a Command byte now.
      {
        if(SMBV_Opt5 == TWDR)
        {
          TW_state = TW_Wait4RW;	//set up next state
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//enable ACKing
          return;
        }
      }
      //In all cases except those that 'return' (above), it's an error.
      TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
      TWCR = (1<<TWEA) | (1<<TWEN);     //disable int, and DON'T clear the TWINT flag!
      TW_state = TW_IDLE;		//Reset the state machine.
      break;


    case TW_Wait4RW:	//We will now find out if we will RX more, or we need to TX a reply instead.
      if(TWS_RDATA == Status)		//It is a WRITE-type command. Prep the RX buffer to accept more data.
      {	//NOTE: OptionalMfgFunction5 is a BLOCK command in both directions.
      	//Place all bytes of the transaction into the buffer so we can do a PEC on it if needed.
      	TW_RxBuf[0] = TWAR & 0xFE;	//PEC requires the slave address to be included.
      	TW_RxBuf[1] = SMBV_Opt5;	//store the previously-send Command.
        TW_RxBuf[2] = TWDR;		//THIS byte is the block byte count.
        TW_RxBufCnt = TWDR;
        TW_RxBufIndex = 3;		//the index to store data in the buffer
        TW_state = TW_Wait4Data;
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//enable ACKing
      }
      else
      if(TWS_REPEAT == Status)	//We saw a re-Start, so must be getting ready for a Read cmd.
      {	//Must now interpret previously-sent CurrentCmd & set up Reply data.
        TWI_CmdFlags = SMB_SetUpReply;		//Foreground routine will set up TWCR.
        TW_state = TW_ReplyData;		//Move to next state.
        TWCR = (1<<TWEA) | (1<<TWEN);           //disable int, and DON'T clear the TWINT flag!
        return;					
      }
      else  //some type of error!
      {
        TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
        TWCR = (1<<TWEA) | (1<<TWEN);           //disable int, and DON'T clear the TWINT flag!
        TW_state = TW_IDLE;		        //Reset the state machine.
      }
      break;


    case TW_Wait4Data:	//We are in Slave Receive operating mode.
      if(TWS_RDATA == Status)			//received a data byte
      {
        TW_RxBuf[TW_RxBufIndex++] = TWDR;	//store the byte
        if(0 == --TW_RxBufCnt)			//Have we received ALL expected data now?
        {
          TW_RxBufCnt = TW_RxBufIndex;		//re-use the Write Index's value as the Valid Byte Count.
          TW_RxBufIndex = 0;			//clear the index in preparation for interpreting it.
          TWI_CmdFlags = SMB_GotCmdData;	//tell Foreground to process this now.
          //The foreground code is now responsible for either flagging an error and resetting
          // the state to IDLE, or clearing TWINT to allow the transaction to finish with a STOP.
          //If the cmd is OK, the Foregound must also clear TW_RxBufCnt when done so that STOP works right.
          //The foreground code must reference the UsePEC flag to determine if PEC is included & is valid.
          TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
          TW_state = TW_IDLE;		//Expecting a STOP next; just 'eat' it at TW_Idle.
          return;
        }
      }
      else
      if(TWS_RSTOP == Status)		//got a STOP; all done RXing data now.
      { //Note: if we get a STOP prematurely, then we simply ignore the command,
      	//  since it is too late to inform the Master of the error.
        TW_state = TW_IDLE;		//Reset the state machine in all cases.
      }
      else  //some type of error!
      {
        TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
        TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
        TW_state = TW_IDLE;		//Reset the state machine.
        return;
      }
      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//enable ACKing
      break;


    case TW_ReplyData:	//We are now in Slave Transmit operating mode.
      //Note: TW_TxBufCnt *always* includes the PEC byte! Since we don't
      // know whether the Master actually WANTS the PEC byte or not, we will
      // always TRY to send it, regardless of the state of the UsePEC flag.
      // If the Master does NOT want it, we will get a NAK while the PEC
      // byte is still in the buffer.  In the rare case where we send it all,
      // including the PEC byte, but we still get an ACK back, the TWI module
      // will be off-line anyway due to not setting the TWEA bit after sending PEC,
      // and we will therefore be unable to flag that an error has occurred.
      if((TWS_SLA_R == Status) || (TWS_RACK == Status))	//send out Reply data
      {
        TWDR = TW_TxBuf[TW_TxBufIndex++];	//send data out
        if(--TW_TxBufCnt == 0)			//Have we emptied the buffer, incl. PEC?
          TWCR = (1<<TWINT) | (1<<TWEN);		// Yes, so don't set TWEA.
        else
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// No, so assert TWEA.
      }
      else
      if(TWS_RNAK == Status)	//We may have gotten this validly or as an Error.
      {
        if(TW_TxBufCnt == 1)	//Not an error. Master didn't want PEC; clear UsePEC flag!
        {
          TW_TxBufCnt = 0;	//clear the buffer too.
          UsePEC = 0;
        }
        else
        if(TW_TxBufCnt == 0)	//Not an error. Master wanted PEC (and got it); assert UsePEC.
          UsePEC = 1;
        else			//some kind of error occurred; we got NAK too early!
        { //Note: the TWI module is now OFF-LINE, so we can't inform Host of this error!!
          ;
        }
        TW_state = TW_IDLE;			//In all cases, go back to IDLE.
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
      }
      else
//    if(TWS_FINAL == Status)	//ERROR: we got an ACK but we have no more data!
      { //Since the TWI module is now in "Not Addressed Slave" mode, we can't flag the error.
        TW_state = TW_IDLE;	//Reset the state machine.
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
      }
      break;

  } // end of switch(TW_state)
}










/* *************************************************************************
 *
 *   ISP-over-SMBus Command Interpreter
 *
 ************************************************************************* */


void SMB_BusTimeout(void)
{
  int i;

  for(i=7000; i!=0; i--);   //4 cycles per interation, 28,000 cycles

  SMB_RestoreBus();
}



void SMB_Reply(void)
{
  unsigned char temp;

  TWI_CmdFlags = 0;			//clear the flag that brought us here.
  TW_TxBufIndex = 0;			//initialize
  TW_TxBufCnt = 0;			//initialize

  //At this time, the only valid response is to send back 'Status'.
  TW_TxBuf[0] = 1;                      //SMBus 'block' byte count
  TW_TxBuf[1] = Status;                 //Status value
  TW_TxBufIndex = 0;                    //point back to the start
  TW_TxBufCnt = 2;                      // # of valid bytes in this buffer

  //Generate PEC now for the *entire* transaction, including the original request!
  temp = FastCRC(0, (TWAR & 0xFE));	//use the SLA+W address
  temp = FastCRC(temp, SMBV_Opt5);
  temp = FastCRC(temp, (TWAR | 1));	//use the SLA+R address

  do {temp = FastCRC(temp, TW_TxBuf[TW_TxBufIndex++]);}
    while(TW_TxBufIndex != TW_TxBufCnt);

  TW_TxBuf[TW_TxBufIndex] = temp;	//append the CRC value on the end.
  TW_TxBufCnt++;			//increase the byte count for the PEC.
  TW_TxBufIndex = 0;		        //Reset the buffer pointer.
  TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//have TWI continue from where it stalled.
}





unsigned char EEbusy(void)
{
  return (EECR & (1<<EEPE));
}



unsigned char EEget(unsigned int address)
{
  while(EECR & (1<<EEPE));
  EEAR = address;
  EECR |= (1<<EERE);
  return EEDR;
}

void EEerase(unsigned int address)
{
  while(EECR & (1<<EEPE));
  EEAR = address;
  EECR = (1<<EEPM0) | (1<<EEMPE);
  EECR = (1<<EEPM0) | (1<<EEMPE) | (1<<EEPE);
}

void EEwrite(unsigned int address, unsigned char data)
{
  while(EECR & (1<<EEPE));
  EEAR = address;
  EEDR = data;
  EECR = (1<<EEMPE);                    //do an Erase and a Write
  EECR = (1<<EEMPE) | (1<<EEPE);
}






void LoopMemory(void)
{
  if(LoopFlag == 'E')		//this indicates "Write to EEPROM"
  {
    if(EEbusy())
      return;
    EEwrite(eptr++, SRAMbuffer[dest_i++]);
    if(--ctr)
      return;
    Status = SUCCESS;
    LoopFlag = 0;
    return;
  }
  else
  if(LoopFlag == 'e')		//this indicates "Erase EEPROM"
  {
    if(EEbusy())
      return;
    EEerase(eptr++);
    if(--ctr)
      return;
    Status = SUCCESS;
    LoopFlag = 0;
    return;
  }
}





//This function interprets received commands.
void SMB_CmdInterpreter(void)
{
  unsigned char temp;

  if(UsePEC)				//check the CRC of the received packet.
  {
    temp = 0;				//use this as our CRC value.

    do { temp = FastCRC(temp, TW_RxBuf[TW_RxBufIndex++]); }
    while(TW_RxBufCnt != TW_RxBufIndex);

    if(temp)	//The result of a CRC check SHOULD be =0 if all was ok.
    {
      Status = CRCERROR;
      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//have TWI continue from where it stalled.
      return;
    }
  }
  // Message is valid enough to warrant calling each command's handler now.
  Status = BUSY;                        //in case it's a "looped" operation...


  if(!BigData)
  {
  //  TW_RxBufIndex = 3;			//point to the first byte of Received Data.
    lomemptr = TW_RxBuf[TWRX_LOADDR];
    himemptr = TW_RxBuf[TWRX_HIADDR];
    src_i = TWRX_DATA;
    dest_i = TW_RxBuf[TWRX_OFFSET];
    ctr = TW_RxBuf[TWRX_SIZE];

    temp = TW_RxBuf[TWRX_CMD];          //What is the command?
  }
  else
  {
    temp = 'I';
    src_i = 3;				//for all follow-on blocks, data MUST start at 3!
    //Note that ctr and dest_i are still valid from the previous block.
  }


  if(temp == 'W')                     //Write SRAMbuffer data into memory
  {
    if(TW_RxBuf[TWRX_MEM] == 'F')     //flash?
    {
      src_i = 0;                      //index into SRAMbuffer
      lomemptr &= 0x80;               //make sure the page boundary is clean
      dest_i = lomemptr;
      do
      {
        #pragma diag_suppress=Pe1053  // Suppress warning for conversion from long-type address to flash ptr.
        _FILL_TEMP_WORD(((himemptr<<8)|dest_i), (SRAMbuffer[src_i]|(SRAMbuffer[src_i+1]<<8)));
        #pragma diag_default=Pe1053   // Back to default.
        src_i += 2;                   // Select next word from SRAMbuffer.
        dest_i += 2;                  // Update flash page pointer.
      }
      while(src_i < 0x80);          // Loop until all bytes written.

      _PAGE_WRITE((himemptr<<8)|lomemptr);
      _WAIT_FOR_SPM();
      _ENABLE_RWW_SECTION();

      Status = SUCCESS;
    }
    else
    if(TW_RxBuf[TWRX_MEM] == 'E')  //eeprom?
    {
      eptr = (himemptr<<8) | lomemptr;
      if((eptr + ctr) > 512)
        Status = BADPARAM;
      else
      {
        LoopFlag = 'E';		//this indicates "Write to EEPROM"
      }
    }
    else
      Status = BADPARAM;
  }

  else
  if(temp == 'V')       //Verify info in SRAMbuffer against memory
  {
    if(TW_RxBuf[TWRX_MEM] == 'F')  //flash?
    {
      src_i = SUCCESS;              //use this variable to hold status
      dest_i = 0;
      fptr = (unsigned char __flash *)(lomemptr | (himemptr<<8));
      do { if(SRAMbuffer[dest_i++] != *fptr++) src_i=FAILURE; }
      while (dest_i < 128);
      Status = src_i;
    }
    else
    if(TW_RxBuf[TWRX_MEM] == 'E')  //eeprom?
    {
      eptr = (himemptr<<8) | lomemptr;
      if((eptr + ctr) > 512)
        Status = BADPARAM;
      else
      {
        src_i = SUCCESS;            //use this variable to hold status
        do { if(SRAMbuffer[dest_i++] != EEget(eptr++)) src_i=FAILURE; }
        while (--ctr);
        Status = src_i;
      }
    }
    else
      Status = BADPARAM;
  }

  else
  if(temp == 'E')       //Erase memory, starting at the specified address
  {
    if(TW_RxBuf[TWRX_MEM] == 'F')  //flash?
    {
      _WAIT_FOR_SPM();
      #pragma diag_suppress=Pe1053 // Suppress warning for conversion from long-type address to flash ptr.
      _PAGE_ERASE( (himemptr<<8) | (lomemptr & 0x80) );
      #pragma diag_default=Pe1053 // Back to default.

      Status = SUCCESS;
    }
    else
    if(TW_RxBuf[TWRX_MEM] == 'E')  //eeprom?
    {
      eptr = (himemptr<<8) | lomemptr;
      if((eptr + ctr) > 512)
        Status = BADPARAM;
      else
      {
        LoopFlag = 'e';		//this indicates "erase EEPROM"
      }
    }
    else
      Status = BADPARAM;
  }

  else
  if(temp == 'P')       //prepare to Patch a block of memory by reading it into SRAMbuffer
  {
    if(TW_RxBuf[TWRX_MEM] == 'F')  //flash?
    {
      if(lomemptr & 0x7F)          //it must be on a page boundary!!
        Status = BADPARAM;
      else
      {
        dest_i = 0;
        fptr = (unsigned char __flash *)(lomemptr | (himemptr<<8));
        do {SRAMbuffer[dest_i++] = *fptr++; }
        while (dest_i < 128);
        Status = SUCCESS;
      }
    }
    else
    if(TW_RxBuf[TWRX_MEM] == 'E')  //eeprom?
    {
      eptr = (himemptr<<8) | lomemptr;
      if((eptr + ctr) > 512)
        Status = BADPARAM;
      else
      {
        do { SRAMbuffer[dest_i++] = EEget(eptr++); }
        while (--ctr);
        Status = SUCCESS;
      }
    }
    else
      Status = BADPARAM;
  }

  else
  if(temp == 'A')       //Activate BootLoader ISP code (ignored in this code)
  {
    Status = SUCCESS;   //no action required, just ignore
  }

  else
  if(temp == 'I')       //Insert data from this packet into SRAMbuffer at the offset specified
  {
    if((ctr+dest_i) > 128)
      Status = BADPARAM;
    else
    {
      if(!BigData)
      {
      	if(ctr > 26)	//26 is the most that can fit in an initial packet
      	{
      	  BigData = 1;
          temp = TW_RxBuf[TWRX_BLKCNT] - 6;
      	}
      	else
      	  temp = ctr;
      }
      else              //this is a chained packet and MAY contain up to 32 bytes of data.
      {
        if(ctr <= TW_RxBuf[TWRX_BLKCNT])
          BigData = 0;
        temp = TW_RxBuf[TWRX_BLKCNT];
      }


      do
      {
      	SRAMbuffer[dest_i++] = TW_RxBuf[src_i++];
      	ctr--;
      }
      while (--temp);
      Status = SUCCESS;
    }
  }

  else
  if(temp == 'w')	//first decrypt the entire SRAM block, THEN write.
  {
    ;	//! \todo  Add code here if encryption is desired.
  }

  else
  if(temp == 'v')	//first decrypt the entire SRAM block, THEN verify.
  {
    ;	//! \todo  Add code here if encryption is desired.
  }

  else
  if(temp == 'X')       //Exit the BootLoader
  {
    asm("jmp 0");
  }

  else	//Not a valid command from our list
  {
    SMB_BusTimeout();	//Generate a bus timeout.
    TW_RxBufIndex = 0;	//Wipe out anything in the buffer, just in case.
    TW_RxBufCnt = 0;			
    return;
  }
  //At this point it looks like everything went OK.
  TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	//have TWI continue from where it stalled.
  return;
}





//This function restores the SMBus & the TWI_ISR state machine to normal after
//  we have deliberately generated a bus timeout error (in order to tell the
//  Master that something was wrong with his last command).
void SMB_RestoreBus(void)
{
  TWCR = 0;			//shut down the peripheral
  TW_state = TW_IDLE;	//force an init of the state machine
  TWAR = 0x16;
  TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);	// | (1<<TWSTO)  re-enable

  //Note that we must be careful not to generate some kind of bus error
  // as a result of waking back up, or we will get into an endless loop
  // by generating another bus timeout if the IDLE state doesn't like
  // something it sees when it comes back to life.
}







/* *************************************************************************
 *
 *   Utilities for SMBus commmunications
 *
 ************************************************************************* */



__flash unsigned char crctable[16] =  {0,0x07,0x0E,0x90, 0x1c,0x1b,0x12,0x15, 0x38,0x3F,0x36,0x31, 0x24,0x23,0x2A,0x2D};
__flash unsigned char crctable2[16] = {0,0x70,0xE0,0x90, 0xC1,0xB1,0x21,0x51, 0x83,0xF3,0x63,0x13, 0x42,0x32,0xA2,0xD2};

unsigned char FastCRC(unsigned char LastCRC, unsigned char newbyte)
{
  unsigned char index;

  index = newbyte;
  index ^= LastCRC;
  index >>= 4;
  LastCRC &= 0x0F;
  LastCRC ^= crctable2[index];

  index = LastCRC;
  index ^= newbyte;
  index &= 0x0F;
  LastCRC &= 0xF0;
  LastCRC ^= crctable[index];

  return(LastCRC);
}

/*  This version doesn't require crctable2[], but requires more shifts.
unsigned char SlowerCRC(unsigned char LastCRC, unsigned char newbyte)
{
  unsigned char index;

  index = newbyte;
  index ^= LastCRC;
  index >>= 4;
  LastCRC <<= 4;
  LastCRC ^= crctable[index];

  index = LastCRC >> 4;
  index ^= newbyte;
  index &= 0x0F;
  LastCRC <<= 4;
  LastCRC ^= crctable[index];

  return(LastCRC);
} */














