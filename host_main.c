#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include "pruss/prussdrv.h"
#include "pruss/pruss_intc_mapping.h"
#include "mio.h"
#include "AM335X_GPIO.h"

// PRU data declarations
#define PRU_NUM0 0
#define PRU_NUM1 1

#define PATH_PRU_BINS "/home/jerry/Projs/C_Makefiles/PRUs/C/DSM2_Decoder_1"

extern volatile unsigned int   *gpio0_setdataout_addr;
extern volatile unsigned int   *gpio1_setdataout_addr;
extern volatile unsigned int   *gpio2_setdataout_addr;
extern volatile unsigned int   *gpio3_setdataout_addr;
extern volatile unsigned int   *gpio0_cleardataout_addr;
extern volatile unsigned int   *gpio1_cleardataout_addr;
extern volatile unsigned int   *gpio2_cleardataout_addr;
extern volatile unsigned int   *gpio3_cleardataout_addr;

//#define GPIO0_26_PIN 0x04000000         /* P8.14 LED_2  */
//#define GPIO1_12_PIN 0x00001000         /* P8.12 LED_3  */
//#define GPIO2_03_PIN 0x00000008         /* P8.8 LED_1 */
//#define GPIO2_04_PIN 0x00000010         /* P8.10 LED_4 */


/* sigint handler */
static volatile unsigned int is_sigint = 0;
static volatile unsigned int is_sigterm = 0;

static void sig_handler (int sig_num)
{
	printf ("***** on_sigint(%d) *****\n", sig_num);

	if (sig_num == SIGINT)
	{
		printf ("***** on_sigint(%d) = SIGINT *****\n", sig_num);
		is_sigint = 1;
	}
	else if (sig_num == SIGTERM)
	{
		printf ("***** on_sigint(%d) = SIGTERM *****\n", sig_num);
		is_sigterm = 1;
	}
}

/* main */
int main (int ac, char** av)
{
	int	ret;
	tpruss_intc_initdata 	pruss_intc_initdata = PRUSS_INTC_INITDATA;

	/* Setup the SIGINT signal handling */
	if (signal(SIGINT, sig_handler) == SIG_ERR)
	{
  		printf ("\n**** Can't start SIGINT handler ****\n");
	}
	if (signal(SIGTERM, sig_handler) == SIG_ERR)
	{
  		printf ("\n**** Can't start SIGTERM handler ****\n");
	}

#if 0
	gpio_fast_init (0);

	/* Setup the GPIO pins */
	/* P8.8 - GPIO2.3 - LED_4 */
	int	gpio_num = 67;
	gpio_export (gpio_num);
	gpio_set_dir (gpio_num, GPIO_OUTPUT);

	/* P8.10 - GPIO2.4 - LED_3 */
	gpio_num = 68;
	gpio_export (gpio_num);
	gpio_set_dir (gpio_num, GPIO_OUTPUT);

	/* P8.12 - GPIO1.12 - LED_2 */
	gpio_num = 44;
	gpio_export (gpio_num);
	gpio_set_dir (gpio_num, GPIO_OUTPUT);

	/* P8.14 - GPIO0.26 - LED_4 */
	gpio_num = 26;
	gpio_export (gpio_num);
	gpio_set_dir (gpio_num, GPIO_OUTPUT);

	/* P9.11 - GPIO0.30 - UART4-Rx */
	gpio_num = 30;
	gpio_export (gpio_num);
	gpio_set_dir (gpio_num, GPIO_INPUT);
#endif

	/* Initialize the PRU */
	/* If this segfaults, make sure you're executing as root. */
	ret = prussdrv_init();
	if (0 != ret)
	{
		printf ("prussdrv_init() failed\n");
		return (ret);
	}

	/* Open PRU event Interrupt */
	ret = prussdrv_open (PRU_EVTOUT_0);
	if (ret)
	{
		printf ("prussdrv_open failed\n");
		return (ret);
	}

	/* Get the PRU interrupt initialized */
	ret = prussdrv_pruintc_init (&pruss_intc_initdata);
	if (ret != 0)
	{
		printf ("prussdrv_pruintc_init() failed\n");
		return (ret);
	}

	/* Write program data from data.bin to pru-0 */
	ret = prussdrv_load_datafile (PRU_NUM0, PATH_PRU_BINS"/pru0_data.bin");
	if (ret < 0)
	{
		printf ("prussdrv_load_datafile(PRU-0) failed\n");
		return (ret);
	}

	/* Load/Execute code on pru-0 */
	prussdrv_exec_program_at (PRU_NUM0, PATH_PRU_BINS"/pru0_text.bin", PRU0_START_ADDR);
	if (ret < 0)
	{
		printf ("prussdrv_exec_program_at(PRU-0) failed\n");
		return (ret);
	}

	/* Wait for PRUs */
	printf ("\tINFO: Pausing for Ctl-C signal.\r\n");
	while (!is_sigint && !is_sigterm)
	{
                usleep(500000);
	}

	printf ("\tINFO: Shutting down PRU-%d.\r\n", (int)PRU_NUM0);
	prussdrv_pru_send_event (ARM_PRU0_INTERRUPT);

	/* Wait until PRU has finished execution */
	printf ("\tINFO: Waiting for HALT command from PRU.\r\n");
	ret = prussdrv_pru_wait_event (PRU_EVTOUT_0);
	printf ("\tINFO: PRU program completed, event number %d\n", ret);

	usleep(100000);

	/* Clear PRU events */
	if (0 != (ret = prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT)))
	{
		printf ("prussdrv_pru_clear_event() failed, result = %d\n", ret);
		return (ret);
	}

	/* Disable PRU-0 and close memory mapping*/
	if (0 != (ret =	prussdrv_pru_disable (PRU_NUM0)))
	{
		printf ("prussdrv_pru_disable() failed\n");
		return (ret);
	}

	if (0 != (ret = prussdrv_exit()))
	{
		printf ("prussdrv_pru_exit() failed\n");
		return (ret);
	}

	return 0;
}
