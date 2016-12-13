/*******************************************************************************
* File Name: CURRENT_SENSE_1.h  
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

#if !defined(CY_PINS_CURRENT_SENSE_1_H) /* Pins CURRENT_SENSE_1_H */
#define CY_PINS_CURRENT_SENSE_1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CURRENT_SENSE_1_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    CURRENT_SENSE_1_Write(uint8 value) ;
void    CURRENT_SENSE_1_SetDriveMode(uint8 mode) ;
uint8   CURRENT_SENSE_1_ReadDataReg(void) ;
uint8   CURRENT_SENSE_1_Read(void) ;
uint8   CURRENT_SENSE_1_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define CURRENT_SENSE_1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define CURRENT_SENSE_1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define CURRENT_SENSE_1_DM_RES_UP          PIN_DM_RES_UP
#define CURRENT_SENSE_1_DM_RES_DWN         PIN_DM_RES_DWN
#define CURRENT_SENSE_1_DM_OD_LO           PIN_DM_OD_LO
#define CURRENT_SENSE_1_DM_OD_HI           PIN_DM_OD_HI
#define CURRENT_SENSE_1_DM_STRONG          PIN_DM_STRONG
#define CURRENT_SENSE_1_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define CURRENT_SENSE_1_MASK               CURRENT_SENSE_1__MASK
#define CURRENT_SENSE_1_SHIFT              CURRENT_SENSE_1__SHIFT
#define CURRENT_SENSE_1_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CURRENT_SENSE_1_PS                     (* (reg8 *) CURRENT_SENSE_1__PS)
/* Data Register */
#define CURRENT_SENSE_1_DR                     (* (reg8 *) CURRENT_SENSE_1__DR)
/* Port Number */
#define CURRENT_SENSE_1_PRT_NUM                (* (reg8 *) CURRENT_SENSE_1__PRT) 
/* Connect to Analog Globals */                                                  
#define CURRENT_SENSE_1_AG                     (* (reg8 *) CURRENT_SENSE_1__AG)                       
/* Analog MUX bux enable */
#define CURRENT_SENSE_1_AMUX                   (* (reg8 *) CURRENT_SENSE_1__AMUX) 
/* Bidirectional Enable */                                                        
#define CURRENT_SENSE_1_BIE                    (* (reg8 *) CURRENT_SENSE_1__BIE)
/* Bit-mask for Aliased Register Access */
#define CURRENT_SENSE_1_BIT_MASK               (* (reg8 *) CURRENT_SENSE_1__BIT_MASK)
/* Bypass Enable */
#define CURRENT_SENSE_1_BYP                    (* (reg8 *) CURRENT_SENSE_1__BYP)
/* Port wide control signals */                                                   
#define CURRENT_SENSE_1_CTL                    (* (reg8 *) CURRENT_SENSE_1__CTL)
/* Drive Modes */
#define CURRENT_SENSE_1_DM0                    (* (reg8 *) CURRENT_SENSE_1__DM0) 
#define CURRENT_SENSE_1_DM1                    (* (reg8 *) CURRENT_SENSE_1__DM1)
#define CURRENT_SENSE_1_DM2                    (* (reg8 *) CURRENT_SENSE_1__DM2) 
/* Input Buffer Disable Override */
#define CURRENT_SENSE_1_INP_DIS                (* (reg8 *) CURRENT_SENSE_1__INP_DIS)
/* LCD Common or Segment Drive */
#define CURRENT_SENSE_1_LCD_COM_SEG            (* (reg8 *) CURRENT_SENSE_1__LCD_COM_SEG)
/* Enable Segment LCD */
#define CURRENT_SENSE_1_LCD_EN                 (* (reg8 *) CURRENT_SENSE_1__LCD_EN)
/* Slew Rate Control */
#define CURRENT_SENSE_1_SLW                    (* (reg8 *) CURRENT_SENSE_1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CURRENT_SENSE_1_PRTDSI__CAPS_SEL       (* (reg8 *) CURRENT_SENSE_1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CURRENT_SENSE_1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CURRENT_SENSE_1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CURRENT_SENSE_1_PRTDSI__OE_SEL0        (* (reg8 *) CURRENT_SENSE_1__PRTDSI__OE_SEL0) 
#define CURRENT_SENSE_1_PRTDSI__OE_SEL1        (* (reg8 *) CURRENT_SENSE_1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CURRENT_SENSE_1_PRTDSI__OUT_SEL0       (* (reg8 *) CURRENT_SENSE_1__PRTDSI__OUT_SEL0) 
#define CURRENT_SENSE_1_PRTDSI__OUT_SEL1       (* (reg8 *) CURRENT_SENSE_1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CURRENT_SENSE_1_PRTDSI__SYNC_OUT       (* (reg8 *) CURRENT_SENSE_1__PRTDSI__SYNC_OUT) 


#if defined(CURRENT_SENSE_1__INTSTAT)  /* Interrupt Registers */

    #define CURRENT_SENSE_1_INTSTAT                (* (reg8 *) CURRENT_SENSE_1__INTSTAT)
    #define CURRENT_SENSE_1_SNAP                   (* (reg8 *) CURRENT_SENSE_1__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins CURRENT_SENSE_1_H */


/* [] END OF FILE */
