/*******************************************************************************
* File Name: VOLTAGE_SENSE.h  
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

#if !defined(CY_PINS_VOLTAGE_SENSE_H) /* Pins VOLTAGE_SENSE_H */
#define CY_PINS_VOLTAGE_SENSE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "VOLTAGE_SENSE_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    VOLTAGE_SENSE_Write(uint8 value) ;
void    VOLTAGE_SENSE_SetDriveMode(uint8 mode) ;
uint8   VOLTAGE_SENSE_ReadDataReg(void) ;
uint8   VOLTAGE_SENSE_Read(void) ;
uint8   VOLTAGE_SENSE_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define VOLTAGE_SENSE_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define VOLTAGE_SENSE_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define VOLTAGE_SENSE_DM_RES_UP          PIN_DM_RES_UP
#define VOLTAGE_SENSE_DM_RES_DWN         PIN_DM_RES_DWN
#define VOLTAGE_SENSE_DM_OD_LO           PIN_DM_OD_LO
#define VOLTAGE_SENSE_DM_OD_HI           PIN_DM_OD_HI
#define VOLTAGE_SENSE_DM_STRONG          PIN_DM_STRONG
#define VOLTAGE_SENSE_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define VOLTAGE_SENSE_MASK               VOLTAGE_SENSE__MASK
#define VOLTAGE_SENSE_SHIFT              VOLTAGE_SENSE__SHIFT
#define VOLTAGE_SENSE_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define VOLTAGE_SENSE_PS                     (* (reg8 *) VOLTAGE_SENSE__PS)
/* Data Register */
#define VOLTAGE_SENSE_DR                     (* (reg8 *) VOLTAGE_SENSE__DR)
/* Port Number */
#define VOLTAGE_SENSE_PRT_NUM                (* (reg8 *) VOLTAGE_SENSE__PRT) 
/* Connect to Analog Globals */                                                  
#define VOLTAGE_SENSE_AG                     (* (reg8 *) VOLTAGE_SENSE__AG)                       
/* Analog MUX bux enable */
#define VOLTAGE_SENSE_AMUX                   (* (reg8 *) VOLTAGE_SENSE__AMUX) 
/* Bidirectional Enable */                                                        
#define VOLTAGE_SENSE_BIE                    (* (reg8 *) VOLTAGE_SENSE__BIE)
/* Bit-mask for Aliased Register Access */
#define VOLTAGE_SENSE_BIT_MASK               (* (reg8 *) VOLTAGE_SENSE__BIT_MASK)
/* Bypass Enable */
#define VOLTAGE_SENSE_BYP                    (* (reg8 *) VOLTAGE_SENSE__BYP)
/* Port wide control signals */                                                   
#define VOLTAGE_SENSE_CTL                    (* (reg8 *) VOLTAGE_SENSE__CTL)
/* Drive Modes */
#define VOLTAGE_SENSE_DM0                    (* (reg8 *) VOLTAGE_SENSE__DM0) 
#define VOLTAGE_SENSE_DM1                    (* (reg8 *) VOLTAGE_SENSE__DM1)
#define VOLTAGE_SENSE_DM2                    (* (reg8 *) VOLTAGE_SENSE__DM2) 
/* Input Buffer Disable Override */
#define VOLTAGE_SENSE_INP_DIS                (* (reg8 *) VOLTAGE_SENSE__INP_DIS)
/* LCD Common or Segment Drive */
#define VOLTAGE_SENSE_LCD_COM_SEG            (* (reg8 *) VOLTAGE_SENSE__LCD_COM_SEG)
/* Enable Segment LCD */
#define VOLTAGE_SENSE_LCD_EN                 (* (reg8 *) VOLTAGE_SENSE__LCD_EN)
/* Slew Rate Control */
#define VOLTAGE_SENSE_SLW                    (* (reg8 *) VOLTAGE_SENSE__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define VOLTAGE_SENSE_PRTDSI__CAPS_SEL       (* (reg8 *) VOLTAGE_SENSE__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define VOLTAGE_SENSE_PRTDSI__DBL_SYNC_IN    (* (reg8 *) VOLTAGE_SENSE__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define VOLTAGE_SENSE_PRTDSI__OE_SEL0        (* (reg8 *) VOLTAGE_SENSE__PRTDSI__OE_SEL0) 
#define VOLTAGE_SENSE_PRTDSI__OE_SEL1        (* (reg8 *) VOLTAGE_SENSE__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define VOLTAGE_SENSE_PRTDSI__OUT_SEL0       (* (reg8 *) VOLTAGE_SENSE__PRTDSI__OUT_SEL0) 
#define VOLTAGE_SENSE_PRTDSI__OUT_SEL1       (* (reg8 *) VOLTAGE_SENSE__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define VOLTAGE_SENSE_PRTDSI__SYNC_OUT       (* (reg8 *) VOLTAGE_SENSE__PRTDSI__SYNC_OUT) 


#if defined(VOLTAGE_SENSE__INTSTAT)  /* Interrupt Registers */

    #define VOLTAGE_SENSE_INTSTAT                (* (reg8 *) VOLTAGE_SENSE__INTSTAT)
    #define VOLTAGE_SENSE_SNAP                   (* (reg8 *) VOLTAGE_SENSE__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins VOLTAGE_SENSE_H */


/* [] END OF FILE */
