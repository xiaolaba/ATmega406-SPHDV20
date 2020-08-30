/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      SMBus implementation.
 *
 *      TWI low-level code & command interpreters.
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
 * $URL: http://revisor.norway.atmel.com/AppsAVR8/avr453_Smart_battery_reference_design/tags/20071112_release/code/smbus.c $
 * $Date: 2007-11-12 10:39:44 +0100 (ma, 12 nov 2007) $  \n
 ******************************************************************************/


// Revision List:
//   June 13, 2005: fixed error in handling receive BLOCK (added use of byte count).
//   June 6, 2005: added clearing of SMLOCK after data successfully sent.
//   June 5, 2005: changed TWBR init value to be 12 instead of 11 to make 100KHz.
//   May 10, 2005: added init of TWBR during Master Transmit mode.



//#include <pgmspace.h>
#include <inavr.h>


#define MODULE_SMBUS		/* ensure that we instantiate our variables in smbus.h */
#include "smbus.h"		//instantiates variables & flash strings

#include "timer.h"		//required for BusFault timer activation
#include "pack.h"		//required for a few calculations
#include "analog.h"		//required for SMBus analog calculations
#include "calibration.h"		//required for SMBus analog calculations
//#include "iom406_320.h"
#include <iom406.h>     // IAR headerfile for Mega406 (EW 410)
#include "interpret.h"          // ONLY include this file in THIS module!
#include "main.h"
#include "pwrmgmt.h"


//extern __flash unsigned char SM_Cmd_Table[0x40][2]; //in interpret.c
typedef unsigned char (*ptr2funcUC_V)(void);
extern ptr2funcUC_V SMB_ReadCmd[];
extern ptr2funcUC_V SMB_WriteCmd[];

//Internally-needed prototypes
unsigned char FastCRC(unsigned char LastCRC, unsigned char newbyte);
//extern unsigned char SMLOCK;


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


/* ************************************************************************* */

unsigned char TEST50US = 0;
unsigned char SMLOCK = 0;	//prevents Master bus grab attempts WHILE BUS IS IN MASTER MODE.

unsigned char TW_MTxBuf[8];	//Master-mode TX buffer
unsigned char TW_MTxBufCnt = 0;	//how many valid bytes are in the buffer
unsigned char TW_MTxBufIndex = 0;

// Note for the buffers below, these must be able to contain:
// Slave Address, SMBus Command, Byte Count (if Block-mode), up to 32 bytes, plus PEC.
unsigned char TW_TxBuf[36];	//must be long enough for any outbound strings
unsigned char TW_TxBufCnt = 0;	//how many valid bytes are in the buffer
unsigned char TW_TxBufIndex = 0;

unsigned char TW_RxBuf[10];	//In Application mode (non-ISP mode), only receive WORD commands.
signed char TW_RxBufCnt = 0;
unsigned char TW_RxBufIndex = 0;


//This byte contains flags from the TWI ISR to tell the Foreground code what to do.
//If this byte is ever non-zero, the foreground code will act on its contents.
//Although it is written by both the ISR and the Foreground code, it does not
//  need to be declared VOLATILE because the SMBus is halted until the foreground
//  code finishes processing the associated command and has cleared this flag byte.
unsigned char TWI_CmdFlags;
  #define SMB_GenBusTimeout 1	/* Tell Foreground to generate a bus timeout, as we saw an error! */
  #define SMB_SetUpReply 2	/* Have Foreground set up TW_TxBuf[]. */
  #define SMB_GotCmdData 4	/* Have Foreground interpret the complete received command. */

unsigned char CurrentCmd = 0xFF;
unsigned char UsePEC = 0;	//PEC usage is disabled by default.




/* *************************************************************************
 *
 *   SMBus Initialization routine
 *
 ************************************************************************* */

void InitSMBus(void)
{
  SMB_RestoreBus();
  TWBCSR = (1<<TWBCIF) | (1<<TWBCIE) | (1<<TWBDT1) | (1<<TWBDT0) | (0<<TWBCIP);
}




/* *************************************************************************
 *
 *   SMBus Wakeup Interrupt
 *
 ************************************************************************* */


//This wakes up a battery from sleep mode into the "On" state, per sbdat110, 4.4.2

#pragma vector = TWI_BUS_CD_vect
__interrupt void TWICD_ISR(void)
{
  //clear bits per sbdat110, 4.4.2
  SMBvariables[SMBV_BattMode][hibyte] &= ~(0xE3);	

  if(TWBCSR & (1<<TWBCIP))	//this int was caused by bus DISCONNECT event.
  {
    TWBCSR &= ~(1<<TWBCIP);
    ChangePowerMode(POWERMODE_IDLE,0);
  }
  else	//this int occurred due to a bus CONNECT event.
  {
    TWBCSR |= (1<<TWBCIP);
    ChangePowerMode(POWERMODE_ACTIVE,0);
  }
}




/* *************************************************************************
 *
 *   SMBus PA6 Pin Change Interrupt (needed for Master Mode)
 *
 ************************************************************************* */


#pragma vector = PCINT0_vect
__interrupt void PCINT0_ISR(void)
{
    TEST50US = 0;     //flag to SMBUS.C that bus activity happened
}




#pragma vector = PCINT1_vect
__interrupt void PCINT1_ISR(void)
{
    ;	//unused
}





/* *************************************************************************
 *
 *   Low-Level SMBus Communications State Machine
 *
 ************************************************************************* */

/*

Slave Mode
-------------



Master Mode
-------------

Master mode is initiated by foreground code.  When a message is available for
transmission via Master mode (typically due to a timer expiring), it
is placed in the TWI Master TX buffer.  However, transmission cannot begin
immediately due to the SMBus requirement that "A bus master is required to
check for bus idle time of 50 us."

To meet this requirement, the AVR's pin-change detection mechanism is used
to monitor for bus idle conditions.  PA5 must be wired externally to the
SMBCLK signal (pin 5 on CONN1) to allow this functionality.  (Alternately,
in the user's production system, any I/O signal with pin-change capability
can be used instead of PA5 if desired.)  PA5 is also used to check for the
SMCLK pin being held low for extended periods by other devices on the bus
as well as checking if the AVR itself doing so locally.

A 3-level system is employed to ensure bus availability.  First, the presence
of a message in the transmit buffer results in execution of the foreground code
that manages Master mode.  Next, if the Slave state machine is not in the
IDLE state, no attempt is made to take control of the bus.  Lastly, PA5 is
checked to be sure it is not currently at a logical zero condition.  At this
point the bus appears to be free, so a flag (called TEST50US) is asserted to
indicate that the 50uS test has now begun; the Pin-Change Interrupt for PA5 is
also enabled.  The foreground SMBus Master mode management code is then exited.
With a clock speed of only 1MHz, 50 uS corresponds to only 50 instructions at
most.  Therefore, when the code is re-entered it is guaranteed that at least
50uS has passed.

When the pin-change interrupt is active, bus activity will
trigger the pin-change interrupt.  The corresponding ISR will clear the TEST50US
flag, indicating that the bus is not free.  Thus, when the foreground code is
re-entered, if the flag is not asserted but there is a message in the buffer,
it is understood that the test has failed and must be started again; thus the
flag will again be asserted and the routine will be exited.

When the foreground code is eventually re-entered with the flag still asserted,
and if the PA5 pin is not low, and the TWI ISR Slave State Machine is in IDLE,
action is immediately taken to begin transmission while the bus is free.  An
additional interlock flag (SMLOCK) is asserted to indicate that Master mode has
been entered by the TWI Hardware, so that the foreground code will not attempt to
repeatedly initiate a transmission for the same message.  (Note that checking
for the IDLE state is required due to the possibility that the ISR could have
been activated just after the flag was last asserted.  Additionally it is possible
that another slave device is halting the bus by keeping SMCLK low for an extended
period, so this needs to also be checked before attempting to take over the bus.)

If the AVR is successful in taking ownership of the bus, at the completion of the
transmission of the desired Slave address the ISR will be activated with a Status
code of 0x??.  As a result, the state machine will now vector into the states
related to Master Mode transmission, and the ISR will handle all further aspects
of the transmission.  Alternately, if the TWI module is unsuccessful in taking over
the bus, the TWI ISR will still be entered but with Status codes that are indicative
of an error having occurred.  In this latter case, the SMLOCK and TEST50US flags will
be cleared to force another bus takeover attempt in the future.

*/



unsigned char TXmsg[4][4];    //leave room for PEC
volatile unsigned char TXmsgHead = 0;
volatile unsigned char TXmsgTail = 0;
volatile unsigned char TXmsgQty = 0;

#define TXmsgEmpty (TXmsgQty == 0)
#define TXmsgFull (TXmsgQty == 4)
#define TXmsgDelete {++TXmsgTail; TXmsgTail &= (4-1);  TXmsgQty--;}

//We will compute the PEC for the message as well.
void MasterInsertMsg(unsigned char addr, unsigned char cmd, unsigned int data)
{ //Note that the Charger address is ALWAYS 0x12! (sbsm100.pdf, 5.2)
  //The Host address is always 0x16.
  //The addr always assumes WRITE.
  //We only do a WORD WRITE.

  unsigned char * ptr = TXmsg[TXmsgHead];

  *ptr++ = addr;
  *ptr++ = cmd;
  *ptr++ = (unsigned char) data;
  *ptr = (unsigned char)(data>>8);

  if(TXmsgFull)
    return;


  ++TXmsgHead;
  TXmsgHead &= (4-1);
  TXmsgQty++;
}





//TWI Interrupt Service Routine
//  This ISR contains a state machine for handling the low-level bus communications.
//  It uses the variable TWI_CmdFlags to communicate the need for message processing
//    to the foreground-based command interpreter.  Whenever it passes control off
//    to the foreground routine, the SMBus is halted.

/* Changes to the operation of the SMBus State Machine as of 6/17/2005 (rgf):

   Since there can be multiple Masters on the bus addressing a Smart Battery,
   namely either a Host or a Charger, it is entirely possible that one Master
   may support PEC while the other does not.  For maximum reliability, we must
   be able to support the reception of PEC on-the-fly regardless of whether it
   was used in the past or not.

   To do so, we determine an expected byte count WITHOUT PEC.  While receiving,
   we decrement this counter and store received bytes until either (a) the counter
   drops below a value of (-1), or (b) we receive a STOP.  If we receive a STOP
   and the counter is not either 0 or -1, this is an error and is flagged as such.
   (Note that AFTER receiving a STOP, it is not possible to flag an error to the
   Host or Charger except through the Battery Status flags.)  If the counter is
   zero, this indicates that PEC is disabled.  If the counter is -1, this indicates
   that PEC is enabled.  The message will be processed with this knowledge.

   However, since the SCL line is not being held low due to TWINT being left asserted,
   a different mechanism is needed to hold off incoming messages to the battery
   until the previous message has been processed.  This is done by turning off
   the generation of ACK on any subsequent bytes and/or messages until the foreground
   code has finished processing the prior message.

   It is crucial therefore that the foreground code does not change the
   value of the TWI_CmdFlags variable from being equal to 'SMB_GotCmdData'
   until after it is completely done with processing of the prior message.

   This mechanism also is valid if a Master attempts to perform a Master Read
   while the battery is busy.  In this case, the second byte from the Master,
   specifically the SMBus Command byte, will not be acknowledged.  The Master
   is thereby required to generate a STOP condition.

*/

//State Machine states
enum /*TWISR_State*/ {TW_IDLE=0, TW_Wait4Stop, TW_Wait4Cmd, TW_Wait4RW, TW_Wait4Data, TW_ReplyData, TW_MSLA_W, TW_MCMD_W, TW_MDATA_W };

unsigned char TWISR_state = TW_IDLE;	//state variable


#pragma vector = TWI_vect
__interrupt void TWI_ISR(void)
{
  static unsigned char TWISR_CmdFeatures = 0;	//Command-related feature flags
  unsigned char Status;
  unsigned char tmp;

  Status = TWSR & 0xF8;		//This identifies what caused the interrupt to fire.

  switch(TWISR_state)
  {
    default:
    case TW_IDLE:	//If not SLA_W or RSTOP, is an error!
      if(TWS_SLA_W == Status)	// saw Slave address match with a Write bit
      {
      	if(TWI_CmdFlags == SMB_GotCmdData)
      	{
      	  //Assert that we're 'busy' to SMBus Master by leaving TWEA *off* until we get a STOP.
      	  //Note that this is 'legal' because we have ALREADY sent an ACK to our Slave Address,
      	  // as is required by the SMBus specification.
      	  TWISR_state = TW_Wait4Stop;
          TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing!
      	  return;
      	}
      	else
          TWISR_state = TW_Wait4Cmd;
      }
      else
      if(TWS_RSTOP == Status)	//Saw a Stop, possibly left over from previous cmd.
      {
         ;			//Everything is probably OK.  Take no action.
      }
      else
      if(TWS_START == Status)	//we have successfully sent a START bit. Handle MASTER mode.
      {
        TWDR = TW_MTxBuf[TW_MTxBufIndex++];
        TWISR_state = TW_MSLA_W;
      }
      else //had some type of error!
      {
        SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
        TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
        TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
        TWISR_state = TW_IDLE;			//Reset the state machine.
        return;
      }
      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
      break;

    case TW_Wait4Stop:
      if(TWS_RSTOP == Status)	//Saw a Stop, possibly left over from previous cmd.
      {
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
        TWISR_state = TW_IDLE;			//Reset the state machine.
      }
      else
        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing yet!
      break;


    //SLAVE-mode states follow.

    case TW_Wait4Cmd:	//upon entry, we expect to have received a Cmd byte.
      if(TWS_RCMD == Status)		//It appears that we have received a Command byte now.
      {
        tmp = TWDR;
        if(tmp <= HIGHEST_SMB_CMD)	//Is the Cmd within valid range?
        {
          CurrentCmd = tmp;		//Save a copy.
          tmp = SM_Cmd_Table[tmp][0];	//Grab the Command Characteristics/Features flags.
          if(tmp & SMBslave)		//Is the Command valid for Slaves?
          {				//The command appears to be valid.
            TWISR_CmdFeatures = tmp;	//Save the Feature flags for use in Wait4RW state.
            TWISR_state = TW_Wait4RW;	//set up next state
            TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
            return;
          }
        }
      }
      //In all cases except those that 'return' (above), it's an error.
      SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
      TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
      TWCR = (1<<TWEA) | (1<<TWEN);     //disable int, and DON'T clear the TWINT flag!
      TWISR_state = TW_IDLE;		//Reset the state machine.
      return;
//    break;


    case TW_Wait4RW:	//We will now find out if we will RX more, or we need to TX a reply instead.
      if(TWS_RDATA == Status)		//It is a WRITE-type command. Prep the RX buffer to accept more data.
      {	//NOTE: except for OptionalMfgFunction5, all WRITE cmds are 2-byte, plus optional PEC.
      	//Place all bytes of the transaction into the buffer so we can do a PEC on it if needed.
      	TW_RxBuf[0] = TWAR & 0xFE;	//store everything incl. the slave address for computing PEC.
      	TW_RxBuf[1] = CurrentCmd;	//store the previously-send Command.
        TW_RxBuf[2] = TWDR;		//store this first DATA byte
        TW_RxBufIndex = 3;		//use RxBufIndex as the index to store data in the buffer.
        if(TWISR_CmdFeatures & SCWW)	//is it a Write-WORD command type?
        {
            TW_RxBufCnt = 1;		//We expect 1 more data byte, and possibly PEC after that.
        }
        else
        if(TWISR_CmdFeatures & SCWG)	//is it a write-BLOCK command (must be OptionalMfgFunction5 then)
        {
          tmp = TWDR;
          if((tmp >= 1) && (tmp <= 32))
            TW_RxBufCnt = TWDR;
          else
          {
            SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;
            TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
            TWCR = (1<<TWEA) | (1<<TWEN);       //disable int, and DON'T clear the TWINT flag!
            TWISR_state = TW_IDLE;		//Reset the state machine.
            return;
          }
        }
        else	//this Command doesn't allow EITHER word OR group/block Writes! It's Read-only!
        {
          SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_AccessDenied;
          TWI_CmdFlags = SMB_GenBusTimeout;	//Not a WRITE-type cmd, so generate a bus timeout.
          TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
          TWISR_state = TW_IDLE;		//Reset the state machine.
          return;
        }
        TWISR_state = TW_Wait4Data;
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
      }
      else
      if(TWS_REPEAT == Status)	//We saw a re-Start, so must be getting ready for a Read cmd.
      {	//Must now interpret previously-sent CurrentCmd & set up Reply data.
        if(TWISR_CmdFeatures & (SCRW | SCRG))	//Is it a 'ReadWord' or 'ReadGroup' command type?
        {
          TWI_CmdFlags = SMB_SetUpReply;	//Foreground decoder will set up TWCR.
          TWISR_state = TW_ReplyData;		//Move to next state.
        }
        else
        {
          SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
          TWI_CmdFlags = SMB_GenBusTimeout;	//Not a READ-type cmd, so generate a bus timeout.
          TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
          TWISR_state = TW_IDLE;		//Reset the state machine.
          return;
        }
        TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
        return;					
      }
      else  //some type of error!
      {
        SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
        TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
        TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
        TWISR_state = TW_IDLE;			//Reset the state machine.
        return;
      }
      break;


    case TW_Wait4Data:	//We are in Slave Receive operating mode.
      if(TWS_RDATA == Status)			//received a data byte
      {
        tmp = TWDR;
        if(--TW_RxBufCnt < -1)			//Are we past the PEC byte and still getting more data?
        {
          SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;	//throw away the data & flag error
          TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
          TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
          TWISR_state = TW_IDLE;			//Reset the state machine.
          return;
        }
        TW_RxBuf[TW_RxBufIndex++] = TWDR;	//store the byte
      }

      else
      if(TWS_RSTOP == Status)			//got a STOP; all done RXing data now.
      { //Note: if we get a STOP prematurely, then we simply ignore the command,
      	//  since it is too late to inform the Master of the error.
      	if((TW_RxBufCnt > 0) || (TW_RxBufCnt < -1))	//We got a premature STOP or too much data; ERROR!
      	{
          SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;	//throw away the data.
        }
        else
        {
          if(0 == TW_RxBufCnt)
            UsePEC = 0;				//there is no PEC coming for this packet.
          else
          if(-1 == TW_RxBufCnt)
            UsePEC = 1;				//PEC was included.

          TW_RxBufCnt = TW_RxBufIndex;		//re-use the Write Index's value as the Valid Byte Count.
          TW_RxBufIndex = 0;			//clear the index in preparation for interpreting it.
          TWI_CmdFlags = SMB_GotCmdData;	//tell Foreground to process this now.
          //Note that when (TWI_CmdFlags == SMB_GotCmdData), TWI ISR will respond with BUSY condition.
        }
        TWISR_state = TW_IDLE;		//Reset the state machine in all cases.
      }

      else  //some type of error during transmission!
      {
        SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
        TWI_CmdFlags = SMB_GenBusTimeout;	//Not a WRITE-type cmd, so generate a bus timeout.
        TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
        TWISR_state = TW_IDLE;			//Reset the state machine.
        return;
      }

      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
      break;


    case TW_ReplyData:	//We are now in Slave Transmit operating mode.
      //The foreground code has set up the response that we are now sending.
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
          TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);		// Yes, so don't set TWEA.
        else
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	// No, so assert TWEA.
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
          SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;	//flag it later
        }
        TWISR_state = TW_IDLE;			//In all cases, go back to IDLE.
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
      }
      else
//    if(TWS_FINAL == Status)	//ERROR: we got an ACK but we have no more data!
      { //Since the TWI module is now in "Not Addressed Slave" mode, we can't flag the error
      	// back to the Master DURING this transaction; we WILL assert an error status though.
        SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;
        TWISR_state = TW_IDLE;	//Reset the state machine.
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
      }
      break;



    //MASTER-mode states follow.

    case TW_MSLA_W:	//we just tried to send the SLAVE ADDRESS in Master Mode.
      if(TWS_WRITE_ACK == Status) 	// we got an ACK back from the desired SLA+W transmission
      {
        TWDR = TW_MTxBuf[TW_MTxBufIndex++];	//send out Cmd
        TWISR_state = TW_MCMD_W;
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
      }
      else  //anything else is Unexpected or an Error result.
      { //We simply delete msgs that couldn't be sent as they will be resent in 10secs anyway.
        TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
        TWISR_state = TW_IDLE;	//Reset the state machine.
        TW_MTxBufCnt = 0;
        TW_MTxBufIndex = 0;
        TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
        SMLOCK = 0;
      }
      break;

    case TW_MCMD_W:	//we just sent a Master COMMAND byte or a Data byte
      if(TWS_TXDATA_ACK == Status) 	// we got an ACK from data we sent.
      {
        if(TW_MTxBufCnt > TW_MTxBufIndex)
        {
          TWDR = TW_MTxBuf[TW_MTxBufIndex++];	//send out Data byte
          TWISR_state = TW_MCMD_W;
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
        }
        else  //we've sent everything in the buffer, so send STOP now.
        {
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
          TWISR_state = TW_IDLE;
          TW_MTxBufCnt = 0;
          TW_MTxBufIndex = 0;
          TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
          SMLOCK = 0;
        }
      }
      else  //Unexpected or Error response.
      {
          TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
          TWISR_state = TW_IDLE;
          TW_MTxBufCnt = 0;
          TW_MTxBufIndex = 0;
          TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
          SMLOCK = 0;
      }
      break;

  } // end of switch(TWISR_state)
}



/* *************************************************************************
 *
 *   Foreground SMBus Command Interpreter
 *
 ************************************************************************* */




//This attempts to initiate Master Mode if a message is in the buffer.
void SMB_Master(void)
{
  unsigned char * ptr;
  unsigned char PEC;

  if(!(TW_MTxBufCnt))   //is there an active message
  {
    if(TXmsgEmpty)
      return;

    ptr = TXmsg[TXmsgTail];
    if(UsePEC)
    {
      PEC = FastCRC( 0,  TW_MTxBuf[0] = *ptr++);
      PEC = FastCRC(PEC, TW_MTxBuf[1] = *ptr++);
      PEC = FastCRC(PEC, TW_MTxBuf[2] = *ptr++);
      PEC = FastCRC(PEC, TW_MTxBuf[3] = *ptr);
      TW_MTxBuf[4] = PEC;
      TW_MTxBufCnt = 5;
    }
    else
    {
      TW_MTxBuf[0] = *ptr++;
      TW_MTxBuf[1] = *ptr++;
      TW_MTxBuf[2] = *ptr++;
      TW_MTxBuf[3] = *ptr;
      TW_MTxBufCnt = 4;
    }

    TW_MTxBufIndex = 0;
    SMLOCK = 0;
    TEST50US = 0;
  }

  if(SMLOCK)  //if asserted, we already started a TX action.
    return;

  if(TEST50US)  //if asserted when get here, we've waited at least 50uS & saw no bus activity.
  {
    __disable_interrupt();
    if((PINA & (1<<6)) && (TWISR_state == TW_IDLE))
    {
      SMLOCK = 1;
      TWBR = 12;   //12 yields 100KHz
      TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE)); //send a START
      PCICR = 0;   //disable PCINT0 / PA6
      TEST50US = 0;   //clear this flag, so that PCINT6 gets re-enabled if must re-try.
    }
    __enable_interrupt();
    return;
  }

  if((PINA & (1<<6)) && (TWISR_state == TW_IDLE))
  {
    TEST50US = 1;
    //enable PinChange Interrupt for PA6 here.  PA6 is PCINT6.
    PCIFR = (1<<PCIF1) | (1<<PCIF0); //clear any flags present
    PCMSK0 = (1<<6);   //enable PA6 pin-change
    PCICR = (1<<PCIE0);   //enable PCINT0
  }
}





void Check50uS(void)
{
  if(TEST50US)
    SMB_Master();
}






void SMB_CmdInterpreter(void)
{
  unsigned char temp;

  if(SMB_GenBusTimeout == TWI_CmdFlags)	//The ISR detected an error condition.
  {
    TWI_CmdFlags = 0;			//clear the flag that brought us here.
    //start the 26mS timer.  When it is done, the Timer handler will re-init the TWI peripheral.
    SetGenericTimer(SMBfaultTimer, 26);
    return;
  }
  else
  if(SMB_SetUpReply == TWI_CmdFlags)	//interpret a 'Read' Command.
  {
    TWI_CmdFlags = 0;			//clear the flag that brought us here.
    TW_TxBufIndex = 0;			//initialize
    TW_TxBufCnt = 0;			//initialize
    if(0 != SMB_ReadCmd[CurrentCmd]())	//After interpreting, was there an error??
    {
      SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnsuptdCommand;
      SetGenericTimer(SMBfaultTimer, 26); //generate a bus timeout error.
      TW_TxBufIndex = 0;		  //Wipe out anything in the buffer, just in case.
      TW_TxBufCnt = 0;			
      return;
    }
    else //generate PEC now for the *entire* transaction, including the original request!
    {
      //Assume (TW_TxBufIndex == 0) and (TW_TxBufCtr == # of bytes of data, not incl PEC).
      temp = FastCRC(0, (TWAR & 0xFE));	//use the SLA+W address
      temp = FastCRC(temp, CurrentCmd);
      temp = FastCRC(temp, (TWAR | 1));	//use the SLA+R address

      do {temp = FastCRC(temp, TW_TxBuf[TW_TxBufIndex++]);}
      while(TW_TxBufIndex != TW_TxBufCnt);

      TW_TxBuf[TW_TxBufIndex] = temp;	//append the CRC value on the end.
      TW_TxBufCnt++;			//increase the byte count for the PEC.
      TW_TxBufIndex = 0;		//Reset the buffer pointer.
      TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//have TWI continue from where it stalled.
    }
    return;
  }
  else
  if(SMB_GotCmdData == TWI_CmdFlags)	//process some received command+data.
  {
    //NOTE: as of 6/17/2005, TWI_CmdFlags should NOT be cleared until we are
    // completely done processing this message, else the TWI ISR will overwrite
    // the RX buffer and won't respond with BUSY as it should.  Note also that
    // the TWI is fully re-enabled when entering here, so we MUST NOT write
    // to any of the TWI h/w registers or we could create havoc.  -rgf

    if(UsePEC)				//check the CRC of the received packet.
    {
      temp = 0;				//use this as our CRC value.

      do { temp = FastCRC(temp, TW_RxBuf[TW_RxBufIndex++]); }
      while(TW_RxBufCnt != TW_RxBufIndex);

      if(temp)	//The result of a CRC check SHOULD be =0 if all was ok.
      {
      	SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;

      	//start the 26mS timer to generate a bus timeout error.
      	SetGenericTimer(SMBfaultTimer, 26);
        TWI_CmdFlags = 0;		//clear the flag that brought us here.
        return;
      }
    }

    //The rcvd message is valid enough to warrant calling the command's handler now.


    //Note that none of the regular SMBus commands use Block Mode to send info
    // *TO* the Smart Battery, so TW_RxBuf[2] will NEVER be a byte count, but
    // will always be DATA.  For OptionalMfgFunction(5), the associated command
    // function itself is aware that [2] is the byte count and that the actual
    // data starts at offset=[3].
    TW_RxBufIndex = 2;			//point to the first byte of Received Data.


    if(0 != SMB_WriteCmd[CurrentCmd]())	//After interpreting, was there an error??
    {
      TW_RxBufIndex = 0;		//Wipe out anything in the buffer, just in case.
      TW_RxBufCnt = 0;			

      //start the 26mS timer to generate a bus timeout error.
      SetGenericTimer(SMBfaultTimer, 26);
      TWI_CmdFlags = 0;			//clear the flag that brought us here.
      return;
    }
    //At this point it looks like everything went OK.
    TWI_CmdFlags = 0;			//clear the flag that brought us here.
    return;
  }
}





//This function restores the SMBus & the TWI_ISR state machine to normal after
//  we have deliberately generated a bus timeout error (in order to tell the
//  Master that something was wrong with his last command).
void SMB_RestoreBus(void)
{
  TWCR = 0;			//shut down the peripheral
  TWISR_state = TW_IDLE;	//force an init of the state machine
  TWAR = BATTERY_ADDR;			
  TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	// | (1<<TWSTO)  re-enable

  //Note that we must be careful not to generate some kind of bus error
  // as a result of waking back up, or we will get into an endless loop
  // by generating another bus timeout if the IDLE state doesn't like
  // something it sees when it comes back to life.
}












/* *************************************************************************
 *
 *   Individual handlers for SMBus READ-type commands.
 *
 *   When any of these functions is called, the SMBus is in a halted state,
 *   pending the completion of the called function.  If an error is detected
 *   while processing any of these, a bus timeout error will have to be
 *   generated to inform the Master that there was a problem.  Otherwise,
 *   if everything is OK, the data that is being requested by the Master
 *   must be set up in the TW_TxBuf buffer for transmission to the Master.
 *   The functions below do not need to calculate the PEC value, as this
 *   will be handled in the function SMB_CmdInterpreter() automatically.
 *
 *   The valid return values for all functions below are:
 *      0: error detected; must generate a bus timeout error.
 *      1: requested data is present in the TW_TxBuf buffer.
 *
 ************************************************************************* */




void FillResponseInt(unsigned int info)
{
  TW_TxBuf[0] = (unsigned char) info;
  TW_TxBuf[1] = (unsigned char) (info >> 8);

  TW_TxBufIndex = 0;
  TW_TxBufCnt = 2;
}


void FillResponseStr(char __flash * source)
{
  unsigned char * dest = TW_TxBuf;
  unsigned char ctr = 0;

  for(;;)
  {
    if(*dest++ = *source++)
      ctr++;
    else
      break;
  }

  TW_TxBufIndex = 0;
  TW_TxBufCnt = ctr;
}




/* ************************************************** */

unsigned char SMBR_MfrAccess(void)	// Cmd # 0
{
  FillResponseInt(SMBvar_int[SMBV_MfrAccess]);
  return 0;
}



unsigned char SMBR_RemCapAlm(void)	// 1
{
  FillResponseInt(SMBvar_int[SMBV_RemCapAlm]);
  return 0;
}


unsigned char SMBR_RemTimeAlm(void)	// 2
{
  FillResponseInt(SMBvar_int[SMBV_RemTimeAlm]);
  return 0;
}


unsigned char SMBR_BattMode(void)	// 3
{
  FillResponseInt(SMBvar_int[SMBV_BattMode]);
  return 0;
}


unsigned char SMBR_AtRate(void)		// 4
{
  FillResponseInt(SMBvar_int[SMBV_AtRate]);
  return 0;
}


unsigned char SMBR_AtRateTTF(void)	// 5
{
  unsigned int temp = AtRateTTF();
  SMBvar_int[SMBV_AtRateTTF] = temp;	//save local copy of result for DEBUG PURPOSES ONLY
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_AtRateTTE(void)	// 6
{
  unsigned int temp = AtRateTTE();
  SMBvar_int[SMBV_AtRateTTE] = temp;
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_AtRateOK(void)	// 7
{
  unsigned int temp = AtRateOK();
  SMBvar_int[SMBV_AtRateOK] = temp;
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_Temperature(void)	//  8
{
  unsigned int temp = GetTemperature();
  SMBvar_int[SMBV_Temperature] = temp;
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_Voltage(void)	//  9
{
  unsigned int volt = GetVoltage();
  SMBvar_int[SMBV_Voltage] = volt;
  FillResponseInt(volt);
  return 0;
}


unsigned char SMBR_Current(void)	// 10
{
  signed int current = Current1Sec();

  SMBvar_int[SMBV_Current] = (unsigned int) current;
  FillResponseInt((unsigned int) current);
  return 0;
}


unsigned char SMBR_AvgCurrent(void)	// 11
{
  signed int current = CCarray_Average();

  SMBvar_int[SMBV_AvgCurrent] = (unsigned int) current;
  FillResponseInt((unsigned int) current);
  return 0;
}


unsigned char SMBR_MaxError(void)	// 12
{
  FillResponseInt(SMBvar_int[SMBV_MaxError] = 0);
  return 0;
}


unsigned char SMBR_RelSOC(void)		// 13
{
  unsigned int temp = RelativeSOC();

  SMBvar_int[SMBV_RelSOC] = temp;
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_AbsSOC(void)		// 14
{
  unsigned int temp = AbsoluteSOC();
  SMBvar_int[SMBV_AbsSOC] = temp;
  FillResponseInt(temp);
  return 0;
}


unsigned char SMBR_RemCap(void)		// 15
{
  unsigned int cap = RemainingCap();
  FillResponseInt(cap);
  SMBvar_int[SMBV_RemCap] = cap;
  return 0;
}


unsigned char SMBR_FullChgCap(void)	// 16
{
  unsigned int cap = FullChgCap();
  FillResponseInt(cap);
  SMBvar_int[SMBV_FullChgCap] = cap;
  return 0;
}


unsigned char SMBR_RunTTE(void)		// 17
{
  unsigned int temp = TimeToEmpty(0);
  FillResponseInt(temp);
  SMBvar_int[SMBV_RunTTE] = temp;
  return 0;
}


unsigned char SMBR_AvgTTE(void)		// 18
{
  unsigned int temp = TimeToEmpty(1);
  FillResponseInt(temp);
  SMBvar_int[SMBV_AvgTTE] = temp;
  return 0;
}


unsigned char SMBR_AvgTTF(void)		// 19
{
  unsigned int temp = AvgTimeToFull();
  FillResponseInt(temp);
  SMBvar_int[SMBV_AvgTTF] = temp;
  return 0;
}



/* ********************************************** */

// These two messages are sent from either a Charger or a Host
unsigned char SMBR_ChgCurrent(void)	// 20
{
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  FillResponseInt(SMBvar_int[SMBV_ChgCurrent]);
  return 0;
}


unsigned char SMBR_ChgVoltage(void)	// 21
{
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  FillResponseInt(SMBvar_int[SMBV_ChgVoltage]);
  return 0;
}


/* ********************************************** */


unsigned char SMBR_BattStatus(void)	// 22
{
  FillResponseInt(SMBvar_int[SMBV_BattStatus]);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;
  return 0;
}


unsigned char SMBR_CycleCount(void)	// 23
{
  FillResponseInt(SMBvar_int[SMBV_CycleCount]);
  return 0;
}


unsigned char SMBR_DesignCap(void)	// 24
{
  unsigned long temp;

  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
    temp = PACK_DESIGNCAPMW;
  else
    temp = PACK_DESIGNCAPC5;

  FillResponseInt(temp);
  SMBvar_int[SMBV_DesignCap] = (unsigned int)temp;
  return 0;
}


unsigned char SMBR_DesignVolt(void)	// 25
{
  FillResponseInt(PACK_NOMINALV);
  SMBvar_int[SMBV_DesignVolt] = PACK_NOMINALV;
  return 0;
}


unsigned char SMBR_SpecInfo(void)	// 26
{
  FillResponseInt(SMBvar_int[SMBV_SpecInfo]);	//! \todo  this value is filled in as const by init.
  return 0;
}


unsigned char SMBR_MfrDate(void)	// 27
{
  FillResponseInt(SMBvar_int[SMBV_MfrDate]);	//! \todo  this value is filled in as const by init.
  return 0;
}


unsigned char SMBR_SerialNo(void)	// 28
{
  FillResponseInt(SMBvar_int[SMBV_SerialNo]);	//! \todo  this value is filled in as const by init.
  return 0;

}


unsigned char SMBR_MfrName(void)	// 32
{
  FillResponseStr(str_MfrName);			//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}



unsigned char SMBR_DeviceName(void)	// 33
{
  FillResponseStr(str_DeviceName);		//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


unsigned char SMBR_DeviceChem(void)	// 34
{
  FillResponseStr(str_DeviceChem);		//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


unsigned char SMBR_MfrData(void)	// 35
{
  FillResponseStr(str_MfrData);			//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


unsigned char SMBR_Opt5(void)		// 0x2F
{
  FillResponseInt(12345);			//! \todo  this value is defined as a constant here.
  return 0;
//  return SMBerr_ReservedCommand;
}


unsigned char SMBR_Opt4(void)		// 0x3C (serves as Calibration control)
{
  FillResponseInt( calibration_state ); // Prepare current state in TX buf.
  return 0; // Return "OK, there are data to transmitted".

}


unsigned char SMBR_Opt3(void)		// 0x3D
{

  return SMBerr_ReservedCommand;	//unused
}


unsigned char SMBR_Opt2(void)		// 0x3E
{

  return SMBerr_ReservedCommand;	//unused
}


unsigned char SMBR_Opt1(void)		// 0x3F
{

  return SMBerr_ReservedCommand;	//unused
}


unsigned char SMBR_invalid(void)	//This should never execute, if error is caught early!
{
  return SMBerr_UnsuptdCommand;
}



//typedef unsigned char (*ptr2funcUC_V)(void);

//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcUC_V SMB_ReadCmd[HIGHEST_SMB_CMD+1] =
{
  SMBR_MfrAccess,	//  0
  SMBR_RemCapAlm,	//  1
  SMBR_RemTimeAlm,	//  2
  SMBR_BattMode,  	//  3
  SMBR_AtRate,     	//  4
  SMBR_AtRateTTF,  	//  5
  SMBR_AtRateTTE,  	//  6
  SMBR_AtRateOK,   	//  7
  SMBR_Temperature,	//  8
  SMBR_Voltage,    	//  9
  SMBR_Current,    	// 10
  SMBR_AvgCurrent, 	// 11
  SMBR_MaxError,   	// 12
  SMBR_RelSOC,     	// 13
  SMBR_AbsSOC,     	// 14
  SMBR_RemCap,     	// 15
  SMBR_FullChgCap, 	// 16
  SMBR_RunTTE,     	// 17
  SMBR_AvgTTE,     	// 18
  SMBR_AvgTTF,     	// 19
  SMBR_ChgCurrent, 	// 20
  SMBR_ChgVoltage, 	// 21
  SMBR_BattStatus, 	// 22
  SMBR_CycleCount, 	// 23
  SMBR_DesignCap,  	// 24
  SMBR_DesignVolt, 	// 25
  SMBR_SpecInfo,   	// 26
  SMBR_MfrDate,    	// 27
  SMBR_SerialNo,   	// 28
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_MfrName,    	// 32
  SMBR_DeviceName, 	// 33
  SMBR_DeviceChem, 	// 34
  SMBR_MfrData,    	// 35
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_Opt5,       	// 0x2F
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_Opt4,       	// 0x3C
  SMBR_Opt3,       	// 0x3D
  SMBR_Opt2,       	// 0x3E
  SMBR_Opt1       	// 0x3F
};



/* *************************************************************************
 *
 *   Individual handlers for SMBus WRITE-type commands
 *
 ************************************************************************* */


unsigned char SMBW_MfrAccess(void)	//  0
{
  unsigned char temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_MfrAccess] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


unsigned char SMBW_RemCapAlm(void)	//  1
{
  unsigned char temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_RemCapAlm] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


unsigned char SMBW_RemTimeAlm(void)	//  2
{
  unsigned char temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_RemTimeAlm] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


unsigned char SMBW_BattMode(void)  	//  3
{
  unsigned char tempH = TW_RxBuf[TW_RxBufIndex+1];
  unsigned char tempL;

  tempL = SMBvariables[SMBV_BattMode][lobyte] & 0xF0;

  if(tempH & 0x1C)
    return SMBerr_AccessDenied;		//attempt to write to reserved bits!


  if(~(tempL & INTERNAL_CHARGE_CONTROLLER))
  {
    tempH &= ~CHARGE_CONTROLLER_ENABLED;  //feature not present, don't let it get turned on.
  }


  if(tempH & PRIMARY_BATTERY)
  {
    ;	//let the Host do what it wants with this flag; we ignore it.
  }

  if(tempH & ALARM_MODE)
    SetAlarmMode;	//this DISABLES sending alarm msgs for 60 secs


  if(tempH & CHARGER_MODE)
  {
    ;	//Allow Host to enable us to send Master-mode Charger-Voltage/Current msgs to the Charger
  }


  if(tempH & CAPACITY_MODE)
  {
    ;	//Host must be allowed to control this bit (report in mAH or 10mWH)
  }


  SMBvar_int[SMBV_BattMode] = tempL | (tempH<<8);	//write the modified bits.

  return 0;
}



unsigned char SMBW_AtRate(void)     	//  4
{
  unsigned char temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_AtRate] = temp | (TW_RxBuf[TW_RxBufIndex++]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}

unsigned char SMBW_Opt5(void)       	// 0x2F
{
  __disable_interrupt();
  MCUSR = 0x1F;                         //clear all reset sources before jumping to bootloader code.
//  __watchdog_reset;					//

  asm("jmp 0x9000");					// Byte adress

//  __watchdog_reset;						// reset watchdog
  return(1);							// to avoid compiler warning, should never be reached

//  return SMBerr_ReservedCommand; // <-- Compiler generates warning if return statement is missing.
}

unsigned char SMBW_Opt4(void)       	// 0x3C
{
  calibration_state_req = TW_RxBuf[TW_RxBufIndex++];
  calibration_state_req |= (unsigned int)TW_RxBuf[TW_RxBufIndex++] << 8; // Store request details.
  SetCalibRequest; // Set action flag so that main loop starts calibrating.
  return 0; // Return OK.
}

unsigned char SMBW_Opt3(void)       	// 0x3D
{
  return SMBerr_ReservedCommand;
}

unsigned char SMBW_Opt2(void)       	// 0x3E
{
  return SMBerr_ReservedCommand;
}

unsigned char SMBW_Opt1(void)       	// 0x3F
{
  return SMBerr_ReservedCommand;
}

unsigned char SMBW_Invalid(void)	//This should never execute, if error is caught early!
{
  return SMBerr_AccessDenied;
}



//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcUC_V SMB_WriteCmd[HIGHEST_SMB_CMD+1] =
{
  SMBW_MfrAccess,	//  0
  SMBW_RemCapAlm,	//  1
  SMBW_RemTimeAlm,	//  2
  SMBW_BattMode,  	//  3
  SMBW_AtRate,     	//  4
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,		//0x0F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,		//0x1F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Opt5,       	// 0x2F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Opt4,       	// 0x3C
  SMBW_Opt3,       	// 0x3D
  SMBW_Opt2,       	// 0x3E
  SMBW_Opt1       	// 0x3F
};






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






//! \todo  This can be modified to load defaults from EEPROM rather than fixed values to the SMB variables.
// This sets power-up defaults.
void InitSMBvariables(void)
{
  SMBvar_int[SMBV_MfrAccess] = 0x4060;				// Mega406 ap-note, revision 0 code
  SMBvar_int[SMBV_RemCapAlm] = (PACK_DESIGNCAPTYP / 10);	// per sbdat110, 4.4.1
  SMBvar_int[SMBV_RemTimeAlm] = 0x000A;			// per sbdat110, 4.4.1
  SMBvar_int[SMBV_BattMode  ] = 0x0000;	//
  SMBvar_int[SMBV_AtRate    ] = 0x0000;	//

/* For testing with no calcs
  SMBvar_int[SMBV_AtRateTTF ] = 0x0000;	//
  SMBvar_int[SMBV_AtRateTTE ] = 0x0000;	//
  SMBvar_int[SMBV_AtRateOK  ] = 0x0000;	//

  SMBvar_int[SMBV_Temperature] = 0x0000;	//
  SMBvar_int[SMBV_Voltage    ] = 0x0000;	//
  SMBvar_int[SMBV_Current    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgCurrent ] = 0x0000;	//
  SMBvar_int[SMBV_MaxError   ] = 0x0000;	//
  SMBvar_int[SMBV_RelSOC     ] = 0x0000;	//
  SMBvar_int[SMBV_AbsSOC     ] = 0x0000;	//
  SMBvar_int[SMBV_RemCap     ] = 0x0000;	//

  SMBvar_int[SMBV_FullChgCap] = 0x0000;	//
  SMBvar_int[SMBV_RunTTE    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgTTE    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgTTF    ] = 0x0000;	//
  SMBvar_int[SMBV_ChgCurrent] = 0x0000;	//
  SMBvar_int[SMBV_ChgVoltage] = 0x0000;	//
*/
  SMBvar_int[SMBV_BattStatus] = 0x0080;	//per sbdat110, 4.4.1
  SMBvar_int[SMBV_CycleCount] = 0x0000;	//per sbdat110, 4.4.1

/* For testing with no calcs
  SMBvar_int[SMBV_DesignCap ] = 0x0000;	//
  SMBvar_int[SMBV_DesignVolt] = 0x0000;	//
*/

  SMBvar_int[SMBV_SpecInfo  ] = 0x0031;	// no scaling of I or V; we support PEC, and we're V1.1
  SMBvar_int[SMBV_MfrDate   ] = ((2005-1980)<<9)+(8<<5)+(31);	//! \todo Fill in current year, month, day. Values are octal
  SMBvar_int[SMBV_SerialNo  ] = 12345;	// arbitrary...

  //Note that for the Block-Read variables, we copy those into a RAM buffer only as needed.
  // These are MfrName, DeviceName, DeviceChem, and MfrData.

  SetMaxTopAcc((long)6600*10727);   //! \todo for testing, initialized value before reaching fully charged, 6600mAh * 10727
//  RunningAcc = (long)1000*10727;    //for testing, to start at other point than 0, 1000 * 10727
}




/* Some important notes from the SMBus specs:

Insertion or removal of a Smart Battery may be detected when the ‘Safety Signal’ transitions from or to an
open-circuit value (>100k.)

When an SMBus device acting as the bus master detects an error, it must attempt to return the bus to the idle
state by generating a STOP condition.

The Smart Battery must ALWAYS acknowledge its own address. Failure to do so might cause the SMBus
Host or Smart Battery Charger to incorrectly assume the Smart Battery is NOT present in the system,
although the ‘Safety Signal’ can be used to detect the presence of a Smart Battery in a system. Note
however that the Smart Battery may choose not to acknowledge any byte following its address if it is
busy or otherwise unable to respond. If this occurs, the requestor should re-try the data request.

After each SMBus transaction directed to the Smart Battery device address, the Smart Battery must place
the appropriate error code in the lower nibble of the BatteryStatus() register. If the transaction completed
successfully, the error codes should be cleared to signify that no error was detected. Timeout and other
errors not described by one of the error code types may be signaled with an Unknown Error.

Another device may try to interrogate the battery immediately after a transaction from the Host.
The safest method to insure that the read of BatteryStatus() corresponds to the most recently read data value
is to perform this read of BatteryStatus() immediately after the read of the initial data value. This may be
accomplished by issuing a SMBus START condition after the SMBus STOP condition from the previous
transmission.

A bus master is required to check for bus idle time of 50 us.

The Smart Battery enters the “On State whenever it detects that the SMBus Clock and Data lines go high.
The battery should be active and able to communicate via the SMBus within 1 ms of detecting these SMBus
lines going high.

The Smart Battery may not begin *broadcasting* ChargingVoltage(), ChargingCurrent() or AlarmWarning()
messages to either the SMBus Host or Smart Battery Charger for at least 10 seconds after entering the “On
State.”

When the Smart Battery enters the “On State” the following values must be reinitialized:
Function (Data Value) 		Initial Value 		
---------------------	----------------------------	
BatteryMode() 		Bit 15: CAPACITY_MODE=0
			Bit 14: CHARGER_MODE=0
			Bit 13: ALARM MODE=0
			Bit 9: PRIMARY_BATTERY=0
			Bit 8: CHARGE_CONTROLLER_ENABLED=0


The Smart Battery may enter the “Off State” whenever the SMBus Clock and Data lines both remain low
for greater than 2.5 seconds.

There is no limit to the speed (other than SMBus limits) or rate at which data may be requested from the
Smart Battery. Continuous data polling at high rates is permitted and allowed, though not encouraged due
to limitations on SMBus bandwidth and availability. The Smart Battery may delay any data request by
holding the CLOCK line low for up to 25 ms. This may be done in order to re-calculate the requested data
value or to retrieve data from a storage device.


*/
