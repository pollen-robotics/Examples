/******************************************************************************
 * @file
 * Pin_Mux
 * @TS
 * @version 1.0.0
******************************************************************************/
#ifndef __GPIO_SETUP_H
#define __GPIO_SETUP_H

#include "main.h"

//#define ION_BOARD 
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define G_LED_PORT			GPIOC
#define G_LED_PIN		    GPIO_PIN_9
#define R_LED_PORT          GPIOC
#define R_LED_PIN    		GPIO_PIN_8
#define U_BUTTON_PORT       GPIOA
#define U_BUTTON_PIN    	GPIO_PIN_0

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void GPIO_Setup(void);


#endif // __GPIO_SETUP_H
