/* Notes:
 * This code intended to decode simulated Spectrum DSMX protocol.
 *
 * It is written to run on PRU-0 of a BBBlue, and is intended to read serial input
 * from P9.11 which is routed through GPIO0.30
 * Make certain P9.11 is exported & configured as in input in host_main.c
 *
 *
 * ----- Run with BBG-PRUS-1G & BBG-GPIO-1E device overlays loaded -----
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "linux_types.h"

// Leave this defined for simulated DX6e traffic.
#define USE_SIM_SMP4648 1

#ifndef USE_SIM_SMP4648
  #define USE_GPIO0_30 1
  #define RX_BAUD_RATE 115200
#else
  #define RX_BAUD_RATE 115200
#endif

/* Shared memory used for ring buffer PWM output - used by pru_sbus.h */
#define DPRAM_SHARED 0x00012000

/* Note: PRU number should be defined prior to pru specific headers */
#define PRU0
#include "pru_defs.h"
#include "pru_sbus.h"
#include "pru_hal.h"

#define GPIO0_START_ADDR 0x44E07000
#define GPIO1_START_ADDR 0x4804C000
#define GPIO2_START_ADDR 0x481AC000
#define GPIO3_START_ADDR 0x481ae000

#define GPIO_OE 0x134
#define GPIO_DATAIN (0x138)
#define GPIO_DATAOUT (0x13C)
#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT (0x194)

#define GPIO1_CLR_DATA (GPIO1_START_ADDR | GPIO_CLEARDATAOUT)
#define GPIO1_SET_DATA (GPIO1_START_ADDR | GPIO_SETDATAOUT)

/* All 4 LEDs reside on GPIO1 */
#define BIT_USER_LED1 (1l << 0x15)  /* GPIO1.21 */
#define BIT_USER_LED2 (1l << 0x16)  /* GPIO1.22 */
#define BIT_USER_LED3 (1l << 0x17)  /* GPIO1.23 */
#define BIT_USER_LED4 (1l << 0x18)  /* GPIO1.24 */

/* This field must also be changed in host_main.c */
#define nPruNum 0
#define PRU0_ARM_INTERRUPT 19
#define PRU1_ARM_INTERRUPT 20

/* Baud rate constant */
#ifdef RX_BAUD_RATE
  /* The following constants are intended for 19.2 KHz. traffic */
  #if (RX_BAUD_RATE == 19200)
  const uint32_t	tm_poll[] = {5208, 15625, 26042, 36458, 46875, 57292,
	  			     67708, 78125, 88542, 98958, 109375 };
  /* The following constants are intended for 57.6 KHz. traffic */
  #elif (RX_BAUD_RATE == 57600)
  const uint32_t	tm_poll[] = { 1736, 5208, 8681, 12153, 15625, 19097,
				      22569, 26042, 29514, 32986, 36458 };
  /* The following constants are intended to decode 115.2 kHz. Spectrum DSMX protocol */
  #elif (RX_BAUD_RATE == 115200)
  const uint32_t	tm_poll[] = { 868, 2604, 4340, 6076, 7812, 9549,
			   	      11285, 13021, 14757, 16493, 18229 };
  /* The following constants are intended to decode 125 kHz. Spectrum DSMX protocol */
  #elif (RX_BAUD_RATE == 125000)
  const uint32_t	tm_poll[] = { 800, 2400, 4000, 5600, 7200, 8800,
				      10400, 12000, 13600, 15200, 16800 };
  #endif
#else
  #error Please #define a baudrate to use for Rx input.
#endif

/* State machine definitions */
typedef enum
{
	ST_LookForIdle,
	ST_LookForStart,
	ST_ConfirmStart,
	ST_ReadData,
	ST_ReadStopOne,
} PRIMARY_STATE;


static inline bool read_rx_pin (void)
{
	int32_t	u32_reg_val;

	/* Read P9.11 - GPIO0.30 - Uart4 Rx */
	u32_reg_val = *((volatile unsigned long *)0x44E07138);

	return (u32_reg_val & 0x40000000);
}

static inline u32 read_PIEP_COUNT(void)
{
    return PIEP_COUNT;
}

/* Function to PWM commands int ring buffer
 *
 *	struct ring_buffer {
 *	   volatile uint16_t ring_head;
 *	   volatile uint16_t ring_tail;
 *	   struct {
 *	          uint16_t pin_value;
 *	          uint16_t delta_t;
 *	   } buffer[NUM_RING_ENTRIES];
 *	};
 */
void add_to_ring_buffer (uint16_t val, uint16_t delta_tm)
{
	RBUFF->buffer[RBUFF->ring_tail].pin_value = val;
	RBUFF->buffer[RBUFF->ring_tail].delta_t = delta_tm;
	RBUFF->ring_tail = (RBUFF->ring_tail + 1) % NUM_RING_ENTRIES;
}

int main (void)
{
	PRIMARY_STATE 		ePrimState = ST_LookForIdle;
	volatile bool		pin_val, last_pin_val = true;
	volatile uint32_t	tm_start_detected, tm_elapsed, tm_now;
	uint32_t 		tm_last_byte;
	uint8_t 		u8_rx_data; //, num_high_bits;
	uint8_t 		data_bit_index, data_bit_mask;
	bool			bStopOne;

	volatile uint32_t	lSameValCnt;
	uint32_t		lLinkActiveCount = 0;
	uint16_t		nLedByteCount = 0;
	uint32_t		lLedCount = 0;
	bool			bLedState = false;
	bool			bDone = false;

	/* PRU Initialisation */
	PRUCFG_SYSCFG &= ~SYSCFG_STANDBY_INIT;
	PRUCFG_SYSCFG = (PRUCFG_SYSCFG &
					~(SYSCFG_IDLE_MODE_M | SYSCFG_STANDBY_MODE_M)) |
					SYSCFG_IDLE_MODE_NO | SYSCFG_STANDBY_MODE_NO;

	/* Our PRU wins arbitration ???*/
	PRUCFG_SPP |=  SPP_PRU1_PAD_HP_EN;

    	/* Configure timer */
	PIEP_GLOBAL_CFG = GLOBAL_CFG_DEFAULT_INC(1) | GLOBAL_CFG_CMP_INC(1);
	PIEP_CMP_STATUS = CMD_STATUS_CMP_HIT(1); /* clear the interrupt */
	PIEP_CMP_CMP1   = 0x0;
	PIEP_CMP_CFG |= CMP_CFG_CMP_EN(1);
	PIEP_GLOBAL_CFG |= GLOBAL_CFG_CNT_ENABLE;

	/* Initialize in/out pointers for ring buffer */
	RBUFF->ring_head = 0;
	RBUFF->ring_tail = 0;

	add_to_ring_buffer (1234, 4321);
	add_to_ring_buffer (2345, 5432);
	add_to_ring_buffer (0, 0);

	/* primary loop */
	while (!bDone)
	{
		/* Manage user status LED */
		if ((lLinkActiveCount == 0) && (1050000 < ++lLedCount))
		{
			if (bLedState)
			{
				/* Clear LED4 */
				*((volatile uint32_t *)GPIO1_CLR_DATA) = (uint32_t)BIT_USER_LED4;
				bLedState = false;
			}
			else
			{
				/* Set LED4 */
				*((volatile uint32_t *)GPIO1_SET_DATA) = (uint32_t)BIT_USER_LED4;
				bLedState = true;
			}
			lLedCount = 0;
		}

		/* Manage link states */
		switch (ePrimState)
		{
			case ST_LookForIdle:
				pin_val = read_rx_pin();
				if (last_pin_val && pin_val)
				{
//					if (42000 < (++lSameValCnt))	/* ~ 10 ms */
					if (21000 < (++lSameValCnt))	/* ~ 5 ms */
//					if (12750 < (++lSameValCnt))	/* ~ 3 ms */
					{
						lSameValCnt = 0;
						ePrimState = ST_LookForStart;
					}
				}
				last_pin_val = pin_val;
				break;

			case ST_LookForStart:
				pin_val = read_rx_pin();
				if (!last_pin_val && !pin_val)
				{
					if (3 < (++lSameValCnt))	/* ~ 1 us */
					{
						tm_start_detected = read_PIEP_COUNT();
						/* Adjust for the time spent aquiring 4 consequtive values */
						tm_start_detected -= 300;
						lSameValCnt = 0;
						ePrimState = ST_ConfirmStart;
					}
				}
				else
				{
					/* decrement link activity counter */
					if (0 <	lLinkActiveCount)
						lLinkActiveCount--;
//					else
//					{
						/* Clear P9.23 - LED4 */
//						*((volatile unsigned long *)0x4804c190) |= 0x00020000;
//						bLedState = false;
//					}
				}
				last_pin_val = pin_val;
				break;

			case ST_ConfirmStart:
				/* 1/2 bit time - confirm start bit here */
				tm_now = read_PIEP_COUNT();
				tm_elapsed = tm_now - tm_start_detected;
				if (tm_poll[0] <= tm_elapsed)
				{
					if (!read_rx_pin())
					{
						u8_rx_data = 0;
//						num_high_bits = 0;
						ePrimState = ST_ReadData;
						data_bit_index = 1;
						data_bit_mask = 0x01;
					}
					else
						ePrimState = ST_LookForIdle;
				}
				break;

			case ST_ReadData:
				/* 1 1/2 - 8 1/2 bit times - read D0-D7 here */
				tm_now = read_PIEP_COUNT();
				tm_elapsed = tm_now - tm_start_detected;
				if (tm_poll[data_bit_index] <= tm_elapsed)
				{
					if (read_rx_pin())
					{
						u8_rx_data |= data_bit_mask;
//						num_high_bits++;	/* Used later for parity check */
					}
					data_bit_mask <<= 1;
					data_bit_index++;
					if (8 < data_bit_index)
					{
						ePrimState = ST_ReadStopOne;
					}
				}
				break;

			case ST_ReadStopOne:
				/* 9 1/2 bit times - read stop bit here */
				tm_now = read_PIEP_COUNT();
				tm_elapsed = tm_now - tm_start_detected;
				if (tm_poll[9] <= tm_elapsed)
				{
					pin_val = read_rx_pin();
					if (pin_val)
						bStopOne = true;
					else
						bStopOne = false;

					/* Start with new byte */
					last_pin_val = pin_val;
					ePrimState = ST_LookForStart;

					/* Make sure all conditions are met */
					if (bStopOne)
					{
						/* Time delta between bytes in us. */
						tm_elapsed = (tm_now - tm_last_byte) / 200;
						tm_last_byte = tm_now;
						add_to_ring_buffer ((uint16_t)tm_elapsed, u8_rx_data);
#if 1
						/* Link active LED flash */
						if (50 < ++nLedByteCount)
						{
							if (bLedState)
							{
								/* Clear LED4 */
								*((volatile uint32_t *)GPIO1_CLR_DATA) = (uint32_t)BIT_USER_LED4;
								bLedState = false;
							}
							else
							{
								/* Set LED4 */
								*((volatile uint32_t *)GPIO1_SET_DATA) = (uint32_t)BIT_USER_LED4;
								bLedState = true;
							}
							nLedByteCount = 0;
						}
						/* This maitains LED activity while positive */
						lLinkActiveCount = 250000;
#endif
					}
					else
					{
						add_to_ring_buffer (0x0000, u8_rx_data);
					}
				}
				break;

			default:
				ePrimState = ST_LookForIdle;
		} /* End of switch (ePrimState) */

		// Exit if we receive a Host->PRU0 interrupt
		if (__R31 & 0x40000000)
			bDone = true;
	};

	/* Shutdown */
	__R31 = PRU0_ARM_INTERRUPT + 16; // PRUEVENT_0 on PRU_R31_VEC_VALID
	__halt();

	return 0;
}
