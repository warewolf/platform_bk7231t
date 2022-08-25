/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */
#include "include.h"
#include "mem_pub.h"

#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "tuya_memory_heap.h"
static uint32_t s_heap_handle = 0;

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/* Allocate the memory for the heap. */
#if configDYNAMIC_HEAP_SIZE
uint8_t *ucHeap;
#elif( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* The application writer has already defined the array used for the RTOS
	heap - probably so it can be placed in a special segment or address. */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/*-----------------------------------------------------------*/

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void prvHeapInit( void );
extern void bk_printf(const char *fmt, ...);
/*-----------------------------------------------------------*/
#if OSMALLOC_STATISTICAL
void *pvPortMalloc_cm(const char *call_func_name, int line, size_t xWantedSize, int need_zero )
#else
void *pvPortMalloc( size_t xWantedSize )
#endif
{
	void *pvReturn = NULL;

	if(0 == s_heap_handle) {
        prvHeapInit();
	}

	if (xWantedSize == 0) {
		xWantedSize = 4;
	}

    //额外加上4字节，与原heap4保持一致(8字节对齐)，否则在scan时有内存踩踏问题
	xWantedSize += 4;
	
    pvReturn = tuya_memory_heap_malloc(s_heap_handle, xWantedSize);

#if OSMALLOC_STATISTICAL
	if(pvReturn && call_func_name) {
	    bk_printf("\r\nm:%p,%d|%s,%d\r\n", pvReturn, xWantedSize, call_func_name, line);
	}
	if(pvReturn && need_zero) {
		os_memset(pvReturn, 0, xWantedSize);
	}
#endif

	return pvReturn;
}

/*-----------------------------------------------------------*/
#if OSMALLOC_STATISTICAL
void *vPortFree_cm(const char *call_func_name, int line, void *pv )
#else
void vPortFree( void *pv )
#endif
{
	if( pv != NULL )
	{
        tuya_memory_heap_free(s_heap_handle, pv);

#if OSMALLOC_STATISTICAL
        if (call_func_name) {
            bk_printf("\r\nf:%p|%s,%d\r\n", pv, call_func_name, line);
        }
#endif
	}
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
    tuya_memory_heap_diagnose(s_heap_handle);
	return tuya_memory_heap_available(s_heap_handle);
}

size_t xPortGetFreeHeapSize2(void)
{
    return tuya_memory_heap_available(s_heap_handle);
}

/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return 0;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

#if configDYNAMIC_HEAP_SIZE
extern unsigned char _empty_ram;

#define HEAP_START_ADDRESS    (void*)&_empty_ram
#define HEAP_END_ADDRESS      (void*)(0x00400000 + 256 * 1024)

static void *prvHeapGetHeaderPointer(void)
{
	return (void *)HEAP_START_ADDRESS;
}

static uint32_t prvHeapGetTotalSize(void)
{
	ASSERT(HEAP_END_ADDRESS > HEAP_START_ADDRESS);
	return (HEAP_END_ADDRESS - HEAP_START_ADDRESS);
}
#endif

static void prvHeapInit( void )
{
	size_t uxAddress;
	size_t xTotalHeapSize;

    heap_context_t ctx = {
        .enter_critical = vTaskSuspendAll,
        .exit_critical = xTaskResumeAll,
        .printf = bk_printf,
    };
    tuya_memory_heap_init(&ctx);

#if configDYNAMIC_HEAP_SIZE
	xTotalHeapSize = prvHeapGetTotalSize();
	ucHeap = prvHeapGetHeaderPointer();

	bk_printf("prvHeapInit-start addr:0x%x, size:%d\r\n", ucHeap, xTotalHeapSize);
#else
	xTotalHeapSize = configTOTAL_HEAP_SIZE;
#endif
	
	/* Ensure the heap starts on a correctly aligned boundary. */
	uxAddress = ( size_t ) ucHeap;

    tuya_memory_heap_create(uxAddress, xTotalHeapSize, &s_heap_handle);
}
/*-----------------------------------------------------------*/
