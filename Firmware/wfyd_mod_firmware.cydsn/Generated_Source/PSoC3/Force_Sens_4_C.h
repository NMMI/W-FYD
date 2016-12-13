/*******************************************************************************
* File Name: Force_Sens_4_C.h  
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

#if !defined(CY_PINS_Force_Sens_4_C_H) /* Pins Force_Sens_4_C_H */
#define CY_PINS_Force_Sens_4_C_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Force_Sens_4_C_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    Force_Sens_4_C_Write(uint8 value) ;
void    Force_Sens_4_C_SetDriveMode(uint8 mode) ;
uint8   Force_Sens_4_C_ReadDataReg(void) ;
uint8   Force_Sens_4_C_Read(void) ;
uint8   Force_Sens_4_C_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Force_Sens_4_C_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Force_Sens_4_C_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Force_Sens_4_C_DM_RES_UP          PIN_DM_RES_UP
#define Force_Sens_4_C_DM_RES_DWN         PIN_DM_RES_DWN
#define Force_Sens_4_C_DM_OD_LO           PIN_DM_OD_LO
#define Force_Sens_4_C_DM_OD_HI           PIN_DM_OD_HI
#define Force_Sens_4_C_DM_STRONG          PIN_DM_STRONG
#define Force_Sens_4_C_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Force_Sens_4_C_MASK               Force_Sens_4_C__MASK
#define Force_Sens_4_C_SHIFT              Force_Sens_4_C__SHIFT
#define Force_Sens_4_C_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Force_Sens_4_C_PS                     (* (reg8 *) Force_Sens_4_C__PS)
/* Data Register */
#define Force_Sens_4_C_DR                     (* (reg8 *) Force_Sens_4_C__DR)
/* Port Number */
#define Force_Sens_4_C_PRT_NUM                (* (reg8 *) Force_Sens_4_C__PRT) 
/* Connect to Analog Globals */                                                  
#define Force_Sens_4_C_AG                     (* (reg8 *) Force_Sens_4_C__AG)                       
/* Analog MUX bux enable */
#define Force_Sens_4_C_AMUX                   (* (reg8 *) Force_Sens_4_C__AMUX) 
/* Bidirectional Enable */                                                        
#define Force_Sens_4_C_BIE                    (* (reg8 *) Force_Sens_4_C__BIE)
/* Bit-mask for Aliased Register Access */
#define Force_Sens_4_C_BIT_MASK               (* (reg8 *) Force_Sens_4_C__BIT_MASK)
/* Bypass Enable */
#define Force_Sens_4_C_BYP                    (* (reg8 *) Force_Sens_4_C__BYP)
/* Port wide control signals */                                                   
#define Force_Sens_4_C_CTL                    (* (reg8 *) Force_Sens_4_C__CTL)
/* Drive Modes */
#define Force_Sens_4_C_DM0                    (* (reg8 *) Force_Sens_4_C__DM0) 
#define Force_Sens_4_C_DM1                    (* (reg8 *) Force_Sens_4_C__DM1)
#define Force_Sens_4_C_DM2                    (* (reg8 *) Force_Sens_4_C__DM2) 
/* Input Buffer Disable Override */
#define Force_Sens_4_C_INP_DIS                (* (reg8 *) Force_Sens_4_C__INP_DIS)
/* LCD Common or Segment Drive */
#define Force_Sens_4_C_LCD_COM_SEG            (* (reg8 *) Force_Sens_4_C__LCD_COM_SEG)
/* Enable Segment LCD */
#define Force_Sens_4_C_LCD_EN                 (* (reg8 *) Force_Sens_4_C__LCD_EN)
/* Slew Rate Control */
#define Force_Sens_4_C_SLW                    (* (reg8 *) Force_Sens_4_C__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Force_Sens_4_C_PRTDSI__CAPS_SEL       (* (reg8 *) Force_Sens_4_C__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Force_Sens_4_C_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Force_Sens_4_C__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Force_Sens_4_C_PRTDSI__OE_SEL0        (* (reg8 *) Force_Sens_4_C__PRTDSI__OE_SEL0) 
#define Force_Sens_4_C_PRTDSI__OE_SEL1        (* (reg8 *) Force_Sens_4_C__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Force_Sens_4_C_PRTDSI__OUT_SEL0       (* (reg8 *) Force_Sens_4_C__PRTDSI__OUT_SEL0) 
#define Force_Sens_4_C_PRTDSI__OUT_SEL1       (* (reg8 *) Force_Sens_4_C__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Force_Sens_4_C_PRTDSI__SYNC_OUT       (* (reg8 *) Force_Sens_4_C__PRTDSI__SYNC_OUT) 


#if defined(Force_Sens_4_C__INTSTAT)  /* Interrupt Registers */

    #define Force_Sens_4_C_INTSTAT                (* (reg8 *) Force_Sens_4_C__INTSTAT)
    #define Force_Sens_4_C_SNAP                   (* (reg8 *) Force_Sens_4_C__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins Force_Sens_4_C_H */


/* [] END OF FILE */
