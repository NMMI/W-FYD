/*******************************************************************************
* File Name: MOTOR_2A.h  
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

#if !defined(CY_PINS_MOTOR_2A_H) /* Pins MOTOR_2A_H */
#define CY_PINS_MOTOR_2A_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "MOTOR_2A_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    MOTOR_2A_Write(uint8 value) ;
void    MOTOR_2A_SetDriveMode(uint8 mode) ;
uint8   MOTOR_2A_ReadDataReg(void) ;
uint8   MOTOR_2A_Read(void) ;
uint8   MOTOR_2A_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define MOTOR_2A_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define MOTOR_2A_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define MOTOR_2A_DM_RES_UP          PIN_DM_RES_UP
#define MOTOR_2A_DM_RES_DWN         PIN_DM_RES_DWN
#define MOTOR_2A_DM_OD_LO           PIN_DM_OD_LO
#define MOTOR_2A_DM_OD_HI           PIN_DM_OD_HI
#define MOTOR_2A_DM_STRONG          PIN_DM_STRONG
#define MOTOR_2A_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define MOTOR_2A_MASK               MOTOR_2A__MASK
#define MOTOR_2A_SHIFT              MOTOR_2A__SHIFT
#define MOTOR_2A_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define MOTOR_2A_PS                     (* (reg8 *) MOTOR_2A__PS)
/* Data Register */
#define MOTOR_2A_DR                     (* (reg8 *) MOTOR_2A__DR)
/* Port Number */
#define MOTOR_2A_PRT_NUM                (* (reg8 *) MOTOR_2A__PRT) 
/* Connect to Analog Globals */                                                  
#define MOTOR_2A_AG                     (* (reg8 *) MOTOR_2A__AG)                       
/* Analog MUX bux enable */
#define MOTOR_2A_AMUX                   (* (reg8 *) MOTOR_2A__AMUX) 
/* Bidirectional Enable */                                                        
#define MOTOR_2A_BIE                    (* (reg8 *) MOTOR_2A__BIE)
/* Bit-mask for Aliased Register Access */
#define MOTOR_2A_BIT_MASK               (* (reg8 *) MOTOR_2A__BIT_MASK)
/* Bypass Enable */
#define MOTOR_2A_BYP                    (* (reg8 *) MOTOR_2A__BYP)
/* Port wide control signals */                                                   
#define MOTOR_2A_CTL                    (* (reg8 *) MOTOR_2A__CTL)
/* Drive Modes */
#define MOTOR_2A_DM0                    (* (reg8 *) MOTOR_2A__DM0) 
#define MOTOR_2A_DM1                    (* (reg8 *) MOTOR_2A__DM1)
#define MOTOR_2A_DM2                    (* (reg8 *) MOTOR_2A__DM2) 
/* Input Buffer Disable Override */
#define MOTOR_2A_INP_DIS                (* (reg8 *) MOTOR_2A__INP_DIS)
/* LCD Common or Segment Drive */
#define MOTOR_2A_LCD_COM_SEG            (* (reg8 *) MOTOR_2A__LCD_COM_SEG)
/* Enable Segment LCD */
#define MOTOR_2A_LCD_EN                 (* (reg8 *) MOTOR_2A__LCD_EN)
/* Slew Rate Control */
#define MOTOR_2A_SLW                    (* (reg8 *) MOTOR_2A__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define MOTOR_2A_PRTDSI__CAPS_SEL       (* (reg8 *) MOTOR_2A__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define MOTOR_2A_PRTDSI__DBL_SYNC_IN    (* (reg8 *) MOTOR_2A__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define MOTOR_2A_PRTDSI__OE_SEL0        (* (reg8 *) MOTOR_2A__PRTDSI__OE_SEL0) 
#define MOTOR_2A_PRTDSI__OE_SEL1        (* (reg8 *) MOTOR_2A__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define MOTOR_2A_PRTDSI__OUT_SEL0       (* (reg8 *) MOTOR_2A__PRTDSI__OUT_SEL0) 
#define MOTOR_2A_PRTDSI__OUT_SEL1       (* (reg8 *) MOTOR_2A__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define MOTOR_2A_PRTDSI__SYNC_OUT       (* (reg8 *) MOTOR_2A__PRTDSI__SYNC_OUT) 


#if defined(MOTOR_2A__INTSTAT)  /* Interrupt Registers */

    #define MOTOR_2A_INTSTAT                (* (reg8 *) MOTOR_2A__INTSTAT)
    #define MOTOR_2A_SNAP                   (* (reg8 *) MOTOR_2A__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins MOTOR_2A_H */


/* [] END OF FILE */
