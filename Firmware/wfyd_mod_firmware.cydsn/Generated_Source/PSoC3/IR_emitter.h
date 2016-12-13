/*******************************************************************************
* File Name: IR_emitter.h  
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

#if !defined(CY_PINS_IR_emitter_H) /* Pins IR_emitter_H */
#define CY_PINS_IR_emitter_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "IR_emitter_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    IR_emitter_Write(uint8 value) ;
void    IR_emitter_SetDriveMode(uint8 mode) ;
uint8   IR_emitter_ReadDataReg(void) ;
uint8   IR_emitter_Read(void) ;
uint8   IR_emitter_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define IR_emitter_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define IR_emitter_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define IR_emitter_DM_RES_UP          PIN_DM_RES_UP
#define IR_emitter_DM_RES_DWN         PIN_DM_RES_DWN
#define IR_emitter_DM_OD_LO           PIN_DM_OD_LO
#define IR_emitter_DM_OD_HI           PIN_DM_OD_HI
#define IR_emitter_DM_STRONG          PIN_DM_STRONG
#define IR_emitter_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define IR_emitter_MASK               IR_emitter__MASK
#define IR_emitter_SHIFT              IR_emitter__SHIFT
#define IR_emitter_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define IR_emitter_PS                     (* (reg8 *) IR_emitter__PS)
/* Data Register */
#define IR_emitter_DR                     (* (reg8 *) IR_emitter__DR)
/* Port Number */
#define IR_emitter_PRT_NUM                (* (reg8 *) IR_emitter__PRT) 
/* Connect to Analog Globals */                                                  
#define IR_emitter_AG                     (* (reg8 *) IR_emitter__AG)                       
/* Analog MUX bux enable */
#define IR_emitter_AMUX                   (* (reg8 *) IR_emitter__AMUX) 
/* Bidirectional Enable */                                                        
#define IR_emitter_BIE                    (* (reg8 *) IR_emitter__BIE)
/* Bit-mask for Aliased Register Access */
#define IR_emitter_BIT_MASK               (* (reg8 *) IR_emitter__BIT_MASK)
/* Bypass Enable */
#define IR_emitter_BYP                    (* (reg8 *) IR_emitter__BYP)
/* Port wide control signals */                                                   
#define IR_emitter_CTL                    (* (reg8 *) IR_emitter__CTL)
/* Drive Modes */
#define IR_emitter_DM0                    (* (reg8 *) IR_emitter__DM0) 
#define IR_emitter_DM1                    (* (reg8 *) IR_emitter__DM1)
#define IR_emitter_DM2                    (* (reg8 *) IR_emitter__DM2) 
/* Input Buffer Disable Override */
#define IR_emitter_INP_DIS                (* (reg8 *) IR_emitter__INP_DIS)
/* LCD Common or Segment Drive */
#define IR_emitter_LCD_COM_SEG            (* (reg8 *) IR_emitter__LCD_COM_SEG)
/* Enable Segment LCD */
#define IR_emitter_LCD_EN                 (* (reg8 *) IR_emitter__LCD_EN)
/* Slew Rate Control */
#define IR_emitter_SLW                    (* (reg8 *) IR_emitter__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define IR_emitter_PRTDSI__CAPS_SEL       (* (reg8 *) IR_emitter__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define IR_emitter_PRTDSI__DBL_SYNC_IN    (* (reg8 *) IR_emitter__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define IR_emitter_PRTDSI__OE_SEL0        (* (reg8 *) IR_emitter__PRTDSI__OE_SEL0) 
#define IR_emitter_PRTDSI__OE_SEL1        (* (reg8 *) IR_emitter__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define IR_emitter_PRTDSI__OUT_SEL0       (* (reg8 *) IR_emitter__PRTDSI__OUT_SEL0) 
#define IR_emitter_PRTDSI__OUT_SEL1       (* (reg8 *) IR_emitter__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define IR_emitter_PRTDSI__SYNC_OUT       (* (reg8 *) IR_emitter__PRTDSI__SYNC_OUT) 


#if defined(IR_emitter__INTSTAT)  /* Interrupt Registers */

    #define IR_emitter_INTSTAT                (* (reg8 *) IR_emitter__INTSTAT)
    #define IR_emitter_SNAP                   (* (reg8 *) IR_emitter__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins IR_emitter_H */


/* [] END OF FILE */
