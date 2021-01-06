/******************************************************************************
 * @file dxl
 * @brief driver example a simple dxl
 * @author Luos
 * @version 0.0.0
 ******************************************************************************/
#ifndef DXL_H
#define DXL_H

#include "luos.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAX_ID 40
#define DXL_TIMEOUT 2
#define TEMP_REFRESH_MS 1000
#define FACTORY_RESET_REG 0xFF
/*******************************************************************************
 * Variables
 ******************************************************************************/

// typedef enum
// {
//     MODE_WHEEL,
//     MODE_ANGLE,
//     MODE_ANGLE_LIMIT,
//     MODE_POWER_LIMIT,
//     MODE_PID,
//     MODE_SPEED,
//     MODE_COMPLIANT,
//     MODE_DETECT,
//     MODE_TEMP,
//     MODE_REG,
//     MODE_ID
// } dxl_mode_t;

/*******************************************************************************
 * Function
 ******************************************************************************/
void Dxl_Init(void);
void Dxl_Loop(void);

#endif /* DXL_H */
