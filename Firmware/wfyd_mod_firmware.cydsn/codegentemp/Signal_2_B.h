/*******************************************************************************
* File Name: Signal_2_B.h  
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

#if !defined(CY_PINS_Signal_2_B_H) /* Pins Signal_2_B_H */
#define CY_PINS_Signal_2_B_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Signal_2_B_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Signal_2_B_Write(uint8 value) ;
void    Signal_2_B_SetDriveMode(uint8 mode) ;
uint8   Signal_2_B_ReadDataReg(void) ;
uint8   Signal_2_B_Read(void) ;
uint8   Signal_2_B_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Signal_2_B_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Signal_2_B_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Signal_2_B_DM_RES_UP          PIN_DM_RES_UP
#define Signal_2_B_DM_RES_DWN         PIN_DM_RES_DWN
#define Signal_2_B_DM_OD_LO           PIN_DM_OD_LO
#define Signal_2_B_DM_OD_HI           PIN_DM_OD_HI
#define Signal_2_B_DM_STRONG          PIN_DM_STRONG
#define Signal_2_B_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Signal_2_B_MASK               Signal_2_B__MASK
#define Signal_2_B_SHIFT              Signal_2_B__SHIFT
#define Signal_2_B_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Signal_2_B_PS                     (* (reg8 *) Signal_2_B__PS)
/* Data Register */
#define Signal_2_B_DR                     (* (reg8 *) Signal_2_B__DR)
/* Port Number */
#define Signal_2_B_PRT_NUM                (* (reg8 *) Signal_2_B__PRT) 
/* Connect to Analog Globals */                                                  
#define Signal_2_B_AG                     (* (reg8 *) Signal_2_B__AG)                       
/* Analog MUX bux enable */
#define Signal_2_B_AMUX                   (* (reg8 *) Signal_2_B__AMUX) 
/* Bidirectional Enable */                                                        
#define Signal_2_B_BIE                    (* (reg8 *) Signal_2_B__BIE)
/* Bit-mask for Aliased Register Access */
#define Signal_2_B_BIT_MASK               (* (reg8 *) Signal_2_B__BIT_MASK)
/* Bypass Enable */
#define Signal_2_B_BYP                    (* (reg8 *) Signal_2_B__BYP)
/* Port wide control signals */                                                   
#define Signal_2_B_CTL                    (* (reg8 *) Signal_2_B__CTL)
/* Drive Modes */
#define Signal_2_B_DM0                    (* (reg8 *) Signal_2_B__DM0) 
#define Signal_2_B_DM1                    (* (reg8 *) Signal_2_B__DM1)
#define Signal_2_B_DM2                    (* (reg8 *) Signal_2_B__DM2) 
/* Input Buffer Disable Override */
#define Signal_2_B_INP_DIS                (* (reg8 *) Signal_2_B__INP_DIS)
/* LCD Common or Segment Drive */
#define Signal_2_B_LCD_COM_SEG            (* (reg8 *) Signal_2_B__LCD_COM_SEG)
/* Enable Segment LCD */
#define Signal_2_B_LCD_EN                 (* (reg8 *) Signal_2_B__LCD_EN)
/* Slew Rate Control */
#define Signal_2_B_SLW                    (* (reg8 *) Signal_2_B__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Signal_2_B_PRTDSI__CAPS_SEL       (* (reg8 *) Signal_2_B__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Signal_2_B_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Signal_2_B__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Signal_2_B_PRTDSI__OE_SEL0        (* (reg8 *) Signal_2_B__PRTDSI__OE_SEL0) 
#define Signal_2_B_PRTDSI__OE_SEL1        (* (reg8 *) Signal_2_B__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Signal_2_B_PRTDSI__OUT_SEL0       (* (reg8 *) Signal_2_B__PRTDSI__OUT_SEL0) 
#define Signal_2_B_PRTDSI__OUT_SEL1       (* (reg8 *) Signal_2_B__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Signal_2_B_PRTDSI__SYNC_OUT       (* (reg8 *) Signal_2_B__PRTDSI__SYNC_OUT) 


#if defined(Signal_2_B__INTSTAT)  /* Interrupt Registers */

    #define Signal_2_B_INTSTAT                (* (reg8 *) Signal_2_B__INTSTAT)
    #define Signal_2_B_SNAP                   (* (reg8 *) Signal_2_B__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Signal_2_B_H */


/* [] END OF FILE */
