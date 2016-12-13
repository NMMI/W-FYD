/*******************************************************************************
* File Name: IR_opto.h  
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

#if !defined(CY_PINS_IR_opto_H) /* Pins IR_opto_H */
#define CY_PINS_IR_opto_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "IR_opto_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    IR_opto_Write(uint8 value) ;
void    IR_opto_SetDriveMode(uint8 mode) ;
uint8   IR_opto_ReadDataReg(void) ;
uint8   IR_opto_Read(void) ;
uint8   IR_opto_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define IR_opto_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define IR_opto_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define IR_opto_DM_RES_UP          PIN_DM_RES_UP
#define IR_opto_DM_RES_DWN         PIN_DM_RES_DWN
#define IR_opto_DM_OD_LO           PIN_DM_OD_LO
#define IR_opto_DM_OD_HI           PIN_DM_OD_HI
#define IR_opto_DM_STRONG          PIN_DM_STRONG
#define IR_opto_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define IR_opto_MASK               IR_opto__MASK
#define IR_opto_SHIFT              IR_opto__SHIFT
#define IR_opto_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define IR_opto_PS                     (* (reg8 *) IR_opto__PS)
/* Data Register */
#define IR_opto_DR                     (* (reg8 *) IR_opto__DR)
/* Port Number */
#define IR_opto_PRT_NUM                (* (reg8 *) IR_opto__PRT) 
/* Connect to Analog Globals */                                                  
#define IR_opto_AG                     (* (reg8 *) IR_opto__AG)                       
/* Analog MUX bux enable */
#define IR_opto_AMUX                   (* (reg8 *) IR_opto__AMUX) 
/* Bidirectional Enable */                                                        
#define IR_opto_BIE                    (* (reg8 *) IR_opto__BIE)
/* Bit-mask for Aliased Register Access */
#define IR_opto_BIT_MASK               (* (reg8 *) IR_opto__BIT_MASK)
/* Bypass Enable */
#define IR_opto_BYP                    (* (reg8 *) IR_opto__BYP)
/* Port wide control signals */                                                   
#define IR_opto_CTL                    (* (reg8 *) IR_opto__CTL)
/* Drive Modes */
#define IR_opto_DM0                    (* (reg8 *) IR_opto__DM0) 
#define IR_opto_DM1                    (* (reg8 *) IR_opto__DM1)
#define IR_opto_DM2                    (* (reg8 *) IR_opto__DM2) 
/* Input Buffer Disable Override */
#define IR_opto_INP_DIS                (* (reg8 *) IR_opto__INP_DIS)
/* LCD Common or Segment Drive */
#define IR_opto_LCD_COM_SEG            (* (reg8 *) IR_opto__LCD_COM_SEG)
/* Enable Segment LCD */
#define IR_opto_LCD_EN                 (* (reg8 *) IR_opto__LCD_EN)
/* Slew Rate Control */
#define IR_opto_SLW                    (* (reg8 *) IR_opto__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define IR_opto_PRTDSI__CAPS_SEL       (* (reg8 *) IR_opto__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define IR_opto_PRTDSI__DBL_SYNC_IN    (* (reg8 *) IR_opto__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define IR_opto_PRTDSI__OE_SEL0        (* (reg8 *) IR_opto__PRTDSI__OE_SEL0) 
#define IR_opto_PRTDSI__OE_SEL1        (* (reg8 *) IR_opto__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define IR_opto_PRTDSI__OUT_SEL0       (* (reg8 *) IR_opto__PRTDSI__OUT_SEL0) 
#define IR_opto_PRTDSI__OUT_SEL1       (* (reg8 *) IR_opto__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define IR_opto_PRTDSI__SYNC_OUT       (* (reg8 *) IR_opto__PRTDSI__SYNC_OUT) 


#if defined(IR_opto__INTSTAT)  /* Interrupt Registers */

    #define IR_opto_INTSTAT                (* (reg8 *) IR_opto__INTSTAT)
    #define IR_opto_SNAP                   (* (reg8 *) IR_opto__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins IR_opto_H */


/* [] END OF FILE */
