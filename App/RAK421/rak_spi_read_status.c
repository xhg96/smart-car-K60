

/**
 * Includes
 */
#include "rak_global.h"
#include "rak411_api.h"
#include "common.h"
#include "include.h"

uint16_t get_status()
{
	uint16_t status=RAK_BUSY;
	 status=spi_Read();	  
		 if(status&SPI_VALID)
		 {}
		 else
			 status=RAK_BUSY;
	return status;
}


uint16_t rak_read_status(void)
{
	
	int16_t					retval;
	int32_t					timeout;
#ifdef RAK_DEBUG_PRINT
	DPRINTF("read status Start\r\n ");
#endif
	timeout=TIMER_NUM;
	RAK_RESET_TIMER1;
	retval=RAK_BUSY	;
	do 
	{
		if (RAK_INC_TIMER_1 > timeout) 
		{			
			retval = RAK_BUSY;
#ifdef RAK_DEBUG_PRINT
			DPRINTF("read status Start timeout\r\n");
#endif
			return retval;
		}
	retval =spi_send_head (READ_STATUS_CMD,0);
    SYS_Delay(HEAD_DELAY);		
	}while(retval != SPI_CMD_ACK);
	retval= get_status();
	return retval;
}
