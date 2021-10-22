#include "stm32f3xx_hal.h"

#include "mb.h" 
#include "mbport.h" 
#include "mbutils.h"
#include "mbtask.h"

USHORT   usRegInputStart = REG_INPUT_START; 
USHORT   usRegInputBuf[REG_INPUT_NREGS]= { 0 ,0};  
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]= { 0, 0, 0, 0, 0, 0 };
/*
REGISTERS:
	3 REGISTERS USED:
	  ADDRESS 0 -> TARGET POSITION IN DEGRESS
	  ADDRESS 1 -> MOVEMENT DURATION IN MILLISECONDS,
	               The target movement duration should be multiple of 9 (e.g. 45, 9000, 999 etc)
	  ADDRESS 2 -> NULL [CAMERA]
	  ADDRESS 3 -> TOF RANGE (mm)
	  ADDRESS 4 -> OBJECT TEMPERATURE (Celsius)
	  ADDRESS 5 -> AMBIENT TEMPERATURE (Celsius)
 */



/*
COILS:
  2 COILS USED:
    ADDRESS 0 -> COMMAND COIL [1 NEW COMMAND, RESET AFTER TEST]
    ADDRESS 1 -> INITIALIZE COIL [1 START INITIALIZE, RESET AFTER TEST]
    ADDRESS 2 -> CAMERA COIL [0 STOP CAMERA, 1 START CAMERA] // TO DOs
*/
unsigned char ucRegCoilsBuf[REG_COILS_SIZE / 8]= { 0 }; /* 16/8=2 */

/*
DISCRETE_INPUTS:
  1 REGISTER USED (2 BIT):
    ADDRESS 0 -> SLAVE_STATUS [0 MOVING, 1 IDLE, 2 NOT_ALIGNED]
    ADDRESS 1 -> ZERO STATE [0 NOT ZERO REACHED, 1 ZERO REACHED]
    ADDRESS 2 -> CAMERA STATE [0 NOT USE, 1 STREAMING]
    ADDRESS 3 -> GRIPPER STATE [0 NOT GRIPPED, 1 GRIPPED] // TO DOs

*/
unsigned char ucRegDiscBuf[REG_DISC_SIZE / 8] = { 0 };  /* 8/8=1 */  



void ModbusRTUTask()
{ 

	eMBErrorCode eStatus = eMBInit( MB_RTU, MB_SLAVE_ADDRESS, 3, 19200, MB_PAR_NONE );
	eStatus = eMBEnable();

} 

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) 
{ 
	eMBErrorCode eStatus = MB_ENOERR; 
	int iRegIndex; 
	if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) ) 
		{ 
			iRegIndex = ( int )( usAddress - usRegInputStart ); 
			while( usNRegs > 0 ) 
				{ 
					*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 ); 
					*pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF ); 
					iRegIndex++; 
					usNRegs--; 
		} 
	} 
	else 
	{ 
		eStatus = MB_ENOREG; 
	} return eStatus; 
} 

eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode ) 
{ 
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
 
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;
 
            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
} 

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode ) 
{ 
eMBErrorCode    eStatus = MB_ENOERR;
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                        xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                        ( unsigned char )( iNCoils >
                                                           8 ? 8 :
                                                           iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
return eStatus;
} 

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete ) 
{ 
 eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                ( unsigned char )( iNDiscrete >
                                                   8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}
