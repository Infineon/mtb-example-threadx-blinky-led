/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for blinky led application example
*              for CYW55xxx Devices.
*
* Related Document: See README.md
*
*

*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg_pins.h"
#include "cyabs_rtos.h"
#include "cy_retarget_io.h"
/*******************************************************************************
* Macros
*******************************************************************************/
#define BLINKY_TASK_NAME            ("Blinky")
#define BLINKY_TASK_STACK_SIZE      1500
#define BLINKY_TASK_PRIORITY        (2)
#define MAIN_TASK_NAME              ("Main_task")
#define MAIN_TASK_STACK_SIZE        1000
#define MAIN_TASK_PRIORITY          (1)

/*Allocate buffer for task stack */
/* Note: Threadx needs 8 byte aligned buffer for stack */
static uint64_t main_task_stack[MAIN_TASK_STACK_SIZE/8];
static uint64_t blinky_task_stack[BLINKY_TASK_STACK_SIZE/8];

/* USER LED toggle period in milliseconds */
#define USER_LED_TOGGLE_PERIOD_MS   1000u

/*Semaphore related define*/
#define INITIAL_COUNT 0
#define MAX_COUNT 1

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* RTOS semaphore */
static cy_semaphore_t xSemaphore;

/* Thread variables*/
static cy_thread_t blinky_task_pointer;
static cy_thread_t main_task_pointer;
/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: blinky_task
********************************************************************************
* Summary:
*  This RTOS task toggles the User LED each time the semaphore is obtained.
*
* Parameters:
*  cy_thread_arg_t arg : Task parameter defined during task creation (unused)
*
* Return:
*  The RTOS task never returns.
*
*******************************************************************************/
static void blinky_task(cy_thread_arg_t arg)
{
    (void) arg;

#ifdef CYBSP_USER_LED
    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, \
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
#endif
    for(;;)
    {
        /* Block until the semaphore is given */
        cy_rtos_semaphore_get(&xSemaphore, CY_RTOS_NEVER_TIMEOUT);

#ifdef CYBSP_USER_LED
        /* Toggle the USER LED state */
        cyhal_gpio_toggle(CYBSP_USER_LED);
#endif
    }
}

/*******************************************************************************
* Function Name: main_task
********************************************************************************
* Summary:
*  This RTOS task releases the semaphore every USER_LED_TOGGLE_PERIOD_MS.
*
* Parameters:
*  cy_thread_arg_t arg : Task parameter defined during task creation (unused)
*
* Return:
*  The RTOS task never returns.
*
*******************************************************************************/
static void main_task(cy_thread_arg_t arg)
{
    (void) arg;

    for(;;)
    {
        /* Block task for USER_LED_TOGGLE_PERIOD_MS. */
        cy_rtos_delay_milliseconds(USER_LED_TOGGLE_PERIOD_MS);

        /* Release semaphore */
        cy_rtos_semaphore_set(&xSemaphore);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1. Initializes the BSP
*    2. Creates other tasks
*    3. Enables Global interrupt
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

int main( )
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        printf("BSP init failed \r\n");
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CY_RETARGET_IO_BAUDRATE);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("cy_retarget_io_init Failed \r\n");
    }

    printf("\x1b[2J\x1b[;H");

    printf("****************** "
              "HAL: Blinky LED Example "
              "****************** \r\n\n");

    /*Set up semaphore for task synchronization*/
    result = cy_rtos_semaphore_init(&xSemaphore,
                                    MAX_COUNT,
                                    INITIAL_COUNT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("semaphore init failed \r\n");
        CY_ASSERT(0);
    }

    /*Create main_task*/
    result = cy_rtos_thread_create(&main_task_pointer,
                                   &main_task,
                                   MAIN_TASK_NAME,
                                   &main_task_stack,
                                   MAIN_TASK_STACK_SIZE,
                                   CY_RTOS_PRIORITY_NORMAL,
                                   0);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Main thread creation failed \r\n");
        CY_ASSERT(0);
    }

    /*Create blinky task*/
    result = cy_rtos_thread_create(&blinky_task_pointer,
                                   &blinky_task,
                                   BLINKY_TASK_NAME,
                                   &blinky_task_stack,
                                   BLINKY_TASK_STACK_SIZE,
                                   CY_RTOS_PRIORITY_NORMAL,
                                   0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Blinky LED thread creation failed \r\n");
        CY_ASSERT(0);
    }
    return 0;

}

/* [] END OF FILE */
