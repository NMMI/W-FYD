/*******************************************************************************
* File Name: servo_motor.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_servo_motor_H) /* Pins servo_motor_H */
#define CY_PINS_servo_motor_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "servo_motor_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    servo_motor_Write(uint8 value) ;
void    servo_motor_SetDriveMode(uint8 mode) ;
uint8   servo_motor_ReadDataReg(void) ;
uint8   servo_motor_Read(void) ;
uint8   servo_motor_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define servo_motor_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define servo_motor_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define servo_motor_DM_RES_UP          PIN_DM_RES_UP
#define servo_motor_DM_RES_DWN         PIN_DM_RES_DWN
#define servo_motor_DM_OD_LO           PIN_DM_OD_LO
#define servo_motor_DM_OD_HI           PIN_DM_OD_HI
#define servo_motor_DM_STRONG          PIN_DM_STRONG
#define servo_motor_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define servo_motor_MASK               servo_motor__MASK
#define servo_motor_SHIFT              servo_motor__SHIFT
#define servo_motor_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define servo_motor_PS                     (* (reg8 *) servo_motor__PS)
/* Data Register */
#define servo_motor_DR                     (* (reg8 *) servo_motor__DR)
/* Port Number */
#define servo_motor_PRT_NUM                (* (reg8 *) servo_motor__PRT) 
/* Connect to Analog Globals */                                                  
#define servo_motor_AG                     (* (reg8 *) servo_motor__AG)                       
/* Analog MUX bux enable */
#define servo_motor_AMUX                   (* (reg8 *) servo_motor__AMUX) 
/* Bidirectional Enable */                                                        
#define servo_motor_BIE                    (* (reg8 *) servo_motor__BIE)
/* Bit-mask for Aliased Register Access */
#define servo_motor_BIT_MASK               (* (reg8 *) servo_motor__BIT_MASK)
/* Bypass Enable */
#define servo_motor_BYP                    (* (reg8 *) servo_motor__BYP)
/* Port wide control signals */                                                   
#define servo_motor_CTL                    (* (reg8 *) servo_motor__CTL)
/* Drive Modes */
#define servo_motor_DM0                    (* (reg8 *) servo_motor__DM0) 
#define servo_motor_DM1                    (* (reg8 *) servo_motor__DM1)
#define servo_motor_DM2                    (* (reg8 *) servo_motor__DM2) 
/* Input Buffer Disable Override */
#define servo_motor_INP_DIS                (* (reg8 *) servo_motor__INP_DIS)
/* LCD Common or Segment Drive */
#define servo_motor_LCD_COM_SEG            (* (reg8 *) servo_motor__LCD_COM_SEG)
/* Enable Segment LCD */
#define servo_motor_LCD_EN                 (* (reg8 *) servo_motor__LCD_EN)
/* Slew Rate Control */
#define servo_motor_SLW                    (* (reg8 *) servo_motor__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define servo_motor_PRTDSI__CAPS_SEL       (* (reg8 *) servo_motor__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define servo_motor_PRTDSI__DBL_SYNC_IN    (* (reg8 *) servo_motor__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define servo_motor_PRTDSI__OE_SEL0        (* (reg8 *) servo_motor__PRTDSI__OE_SEL0) 
#define servo_motor_PRTDSI__OE_SEL1        (* (reg8 *) servo_motor__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define servo_motor_PRTDSI__OUT_SEL0       (* (reg8 *) servo_motor__PRTDSI__OUT_SEL0) 
#define servo_motor_PRTDSI__OUT_SEL1       (* (reg8 *) servo_motor__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define servo_motor_PRTDSI__SYNC_OUT       (* (reg8 *) servo_motor__PRTDSI__SYNC_OUT) 


#if defined(servo_motor__INTSTAT)  /* Interrupt Registers */

    #define servo_motor_INTSTAT                (* (reg8 *) servo_motor__INTSTAT)
    #define servo_motor_SNAP                   (* (reg8 *) servo_motor__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins servo_motor_H */


/* [] END OF FILE */
