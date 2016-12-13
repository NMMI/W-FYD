/*******************************************************************************
* File Name: MOTOR_1A.h  
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

#if !defined(CY_PINS_MOTOR_1A_H) /* Pins MOTOR_1A_H */
#define CY_PINS_MOTOR_1A_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "MOTOR_1A_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MOTOR_1A_Write(uint8 value) ;
void    MOTOR_1A_SetDriveMode(uint8 mode) ;
uint8   MOTOR_1A_ReadDataReg(void) ;
uint8   MOTOR_1A_Read(void) ;
uint8   MOTOR_1A_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MOTOR_1A_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define MOTOR_1A_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define MOTOR_1A_DM_RES_UP          PIN_DM_RES_UP
#define MOTOR_1A_DM_RES_DWN         PIN_DM_RES_DWN
#define MOTOR_1A_DM_OD_LO           PIN_DM_OD_LO
#define MOTOR_1A_DM_OD_HI           PIN_DM_OD_HI
#define MOTOR_1A_DM_STRONG          PIN_DM_STRONG
#define MOTOR_1A_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define MOTOR_1A_MASK               MOTOR_1A__MASK
#define MOTOR_1A_SHIFT              MOTOR_1A__SHIFT
#define MOTOR_1A_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MOTOR_1A_PS                     (* (reg8 *) MOTOR_1A__PS)
/* Data Register */
#define MOTOR_1A_DR                     (* (reg8 *) MOTOR_1A__DR)
/* Port Number */
#define MOTOR_1A_PRT_NUM                (* (reg8 *) MOTOR_1A__PRT) 
/* Connect to Analog Globals */                                                  
#define MOTOR_1A_AG                     (* (reg8 *) MOTOR_1A__AG)                       
/* Analog MUX bux enable */
#define MOTOR_1A_AMUX                   (* (reg8 *) MOTOR_1A__AMUX) 
/* Bidirectional Enable */                                                        
#define MOTOR_1A_BIE                    (* (reg8 *) MOTOR_1A__BIE)
/* Bit-mask for Aliased Register Access */
#define MOTOR_1A_BIT_MASK               (* (reg8 *) MOTOR_1A__BIT_MASK)
/* Bypass Enable */
#define MOTOR_1A_BYP                    (* (reg8 *) MOTOR_1A__BYP)
/* Port wide control signals */                                                   
#define MOTOR_1A_CTL                    (* (reg8 *) MOTOR_1A__CTL)
/* Drive Modes */
#define MOTOR_1A_DM0                    (* (reg8 *) MOTOR_1A__DM0) 
#define MOTOR_1A_DM1                    (* (reg8 *) MOTOR_1A__DM1)
#define MOTOR_1A_DM2                    (* (reg8 *) MOTOR_1A__DM2) 
/* Input Buffer Disable Override */
#define MOTOR_1A_INP_DIS                (* (reg8 *) MOTOR_1A__INP_DIS)
/* LCD Common or Segment Drive */
#define MOTOR_1A_LCD_COM_SEG            (* (reg8 *) MOTOR_1A__LCD_COM_SEG)
/* Enable Segment LCD */
#define MOTOR_1A_LCD_EN                 (* (reg8 *) MOTOR_1A__LCD_EN)
/* Slew Rate Control */
#define MOTOR_1A_SLW                    (* (reg8 *) MOTOR_1A__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define MOTOR_1A_PRTDSI__CAPS_SEL       (* (reg8 *) MOTOR_1A__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define MOTOR_1A_PRTDSI__DBL_SYNC_IN    (* (reg8 *) MOTOR_1A__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define MOTOR_1A_PRTDSI__OE_SEL0        (* (reg8 *) MOTOR_1A__PRTDSI__OE_SEL0) 
#define MOTOR_1A_PRTDSI__OE_SEL1        (* (reg8 *) MOTOR_1A__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define MOTOR_1A_PRTDSI__OUT_SEL0       (* (reg8 *) MOTOR_1A__PRTDSI__OUT_SEL0) 
#define MOTOR_1A_PRTDSI__OUT_SEL1       (* (reg8 *) MOTOR_1A__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define MOTOR_1A_PRTDSI__SYNC_OUT       (* (reg8 *) MOTOR_1A__PRTDSI__SYNC_OUT) 


#if defined(MOTOR_1A__INTSTAT)  /* Interrupt Registers */

    #define MOTOR_1A_INTSTAT                (* (reg8 *) MOTOR_1A__INTSTAT)
    #define MOTOR_1A_SNAP                   (* (reg8 *) MOTOR_1A__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins MOTOR_1A_H */


/* [] END OF FILE */
