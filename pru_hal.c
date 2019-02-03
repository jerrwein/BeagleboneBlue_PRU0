#include <stdint.h>


void ocp_init(void)
{
  /* enable OCP wide access */

  __asm__ __volatile__
  (
   " LBCO &r0, C4, 4, 4 \n"
   " CLR r0, r0, 4 \n"
   " SBCO &r0, C4, 4, 4 \n"
  );
}

void shm_init(unsigned short nPRU)
{
        if (0 == nPRU)
        {
		/* Save R4, R5 which will be used below */
                __asm__ __volatile__
                (
                " SUB r2, r2, 8 \n"
                " SBBO &r4, r2, 0, 8 \n"
                );

		/* CTPPR0 : Constant Table Programmable Pointer Register 0
                   Configure the programmable pointer register for
                   PRU0 (at 0x00022028) by setting:
		   c29_pointer[31:16] field to 0x0000, this will make C29
		   point to 0x49000000 (TPCC).
		   c28_pointer[15:0] field to 0x0120, this will make C28
		   point to 0x00012000 (PRU shared RAM). */
 
                __asm__ __volatile__
                (
                " LDI32 r4, 0x0120 \n"
                " LDI32 r5, 0x22028 \n"
                " SBBO &r4, r5, 0x00, 4 \n"
                );

		/* CTPPR1 : Constant Table Programmable Pointer Register 1
                   Configure the programmable pointer register for
                   PRU0 (at 0x0002202C) by setting: 
                   c31_pointer[31:16] field to 0x0010, this will make C31
                   point to 0x800010000 (EMIF0 DDR Base).
                   c30_pointer[15:0] field to 0x0000, this will make C30
                   point to 0x40000000 (L3 OCMC0). 
		   ----- NOT THOUGHT TO BE NEEDED AT THIS TIME ----- */
                
		/* Restore r4, r5 */
                __asm__ __volatile__
                (
                " LBBO &r4, r2, 0, 8 \n"
                " ADD r2, r2, 8 \n"
                );
        }
        else
        {
                /* Configure the programmable pointer register for */
                /* PRU1 (at 0x00024028) */

                /* Save R4, R5 which will be used below */
                __asm__ __volatile__
                (
                " SUB r2, r2, 8 \n"
                " SBBO &r4, r2, 0, 8 \n"
                );

                __asm__ __volatile__
                (
                " LDI32 r4, 0x0120 \n"
                " LDI32 r5, 0x24028 \n"
                " SBBO &r4, r5, 0x00, 4 \n"
                );

                /* Restore r4, r5 */
                __asm__ __volatile__
                (
                " LBBO &r4, r2, 0, 8 \n"
                " ADD r2, r2, 8 \n"
                );
        }
}

void shm_init2(unsigned short nPRU)
{
	/* Save R4, R5 which will be used below */
	__asm__ __volatile__
	(
	" SUB r2, r2, 8 \n"
	" SBBO &r4, r2, 0, 8 \n"
	);

	if (0 == nPRU)
        {
                /* Configure the programmable pointer register for
                   PRU0 (at 0x00022028) by setting c28_pointer[15:0] field to 0x0120
                   this will make C28 point to 0x00012000 (PRU shared RAM). */

                __asm__ __volatile__
                (
                " LDI32 r4, 0x0120 \n"
                " LDI32 r5, 0x22028 \n"
                " SBBO &r4, r5, 0x00, 4 \n"
                );
	}
        else
        {
                /* Configure the programmable pointer register for
                   PRU1 (at 0x00024028) by setting c28_pointer[15:0] field to 0x0120
                   this will make C28 point to 0x00012000 (PRU shared RAM). */

                __asm__ __volatile__
                (
                " LDI32 r4, 0x0120 \n"
                " LDI32 r5, 0x24028 \n"
                " SBBO &r4, r5, 0x00, 4 \n"
                );
	}

	/* Restore R4, R5 */
	__asm__ __volatile__
	(
	" LBBO &r4, r2, 0, 8 \n"
	" ADD r2, r2, 8 \n"
	);
}

#if 0
void shm_init_pru(unsigned short nPRU)
{
	if (0 == nPRU)
	{
		/* Configure the programmable pointer register for */
		/* PRU0 (at 0x00022028) by setting c28_pointer[15:0] field to 0x0120 */
		/* this will make C28 point to 0x00012000 (PRU shared RAM). */

		/* save R4, R5 which will be used below */
		__asm__ __volatile__
		(
		" SUB r2, r2, 8 \n"
		" SBBO &r4, r2, 0, 8 \n"
		);

		__asm__ __volatile__
		(
		" LDI32 r4, 0x0120 \n"
		" LDI32 r5, 0x22028 \n"
		" SBBO &r4, r5, 0x00, 4 \n"
		);

		/* Restore r4, r5 */
		__asm__ __volatile__
		(
		" LBBO &r4, r2, 0, 8 \n"
		" ADD r2, r2, 8 \n"
		);
	}
	else
	{
		/* Configure the programmable pointer register for */
		/* PRU1 (at 0x00024028) by setting c28_pointer[15:0] field to 0x0120 */
		/* this will make C28 point to 0x00012000 (PRU shared RAM). */

		/* Save R4, R5 which will be used below */
                __asm__ __volatile__
                (
                " SUB r2, r2, 8 \n"
                " SBBO &r4, r2, 0, 8 \n"
                );

                __asm__ __volatile__
                (
                " LDI32 r4, 0x0120 \n"
                " LDI32 r5, 0x24028 \n"
                " SBBO &r4, r5, 0x00, 4 \n"
                );

                /* Restore r4, r5 */
                __asm__ __volatile__
                (
                " LBBO &r4, r2, 0, 8 \n"
                " ADD r2, r2, 8 \n"
                );
	}
}
#endif

void shm_write_uint32(register uint32_t i, register uint32_t x)
{
  /* i is the absolute offset relative from shared memory start */
  /* write x at shm + i */

  __asm__ __volatile__
  (
   " SBCO &r15, C28, r14.w0, 4 \n"
  );
}

void shm_write_float(register uint32_t i, register float x)
{
  __asm__ __volatile__
  (
   " SBCO &r15, C28, r14.w0, 4 \n"
  );
}

uint32_t shm_read(register uint32_t i)
{
  /* i is the absolute offset relative from shared memory start */
  /* read x at shm + i */

  __asm__ __volatile__
  (
//   " LDI32 r0, 0x000000120 \n"
//   " LDI32 r1, 0x22028 \n"
//   " SBBO &r0, r1, 0, 4 \n"

//   " LDI32 r0, 0x00100000 \n"
//   " LDI32 r1, 0x2202c \n"
//   " SBBO &r0, r1, 0, 4 \n"

   " LBCO &r14, C28, r14.w0, 4 \n"
   " JMP R3.w2 \n"
  );

  /* unreached */
  return 0;
}
