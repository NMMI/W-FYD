/*******************************************************************************
* File Name: IR_opto.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "IR_opto.h"


/*******************************************************************************
* Function Name: IR_opto_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void IR_opto_Write(uint8 value) 
{
    uint8 staticBits = (IR_opto_DR & (uint8)(~IR_opto_MASK));
    IR_opto_DR = staticBits | ((uint8)(value << IR_opto_SHIFT) & IR_opto_MASK);
}


/*******************************************************************************
* Function Name: IR_opto_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  IR_opto_DM_STRONG     Strong Drive 
*  IR_opto_DM_OD_HI      Open Drain, Drives High 
*  IR_opto_DM_OD_LO      Open Drain, Drives Low 
*  IR_opto_DM_RES_UP     Resistive Pull Up 
*  IR_opto_DM_RES_DWN    Resistive Pull Down 
*  IR_opto_DM_RES_UPDWN  Resistive Pull Up/Down 
*  IR_opto_DM_DIG_HIZ    High Impedance Digital 
*  IR_opto_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void IR_opto_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(IR_opto_0, mode);
}


/*******************************************************************************
* Function Name: IR_opto_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro IR_opto_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 IR_opto_Read(void) 
{
    return (IR_opto_PS & IR_opto_MASK) >> IR_opto_SHIFT;
}


/*******************************************************************************
* Function Name: IR_opto_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 IR_opto_ReadDataReg(void) 
{
    return (IR_opto_DR & IR_opto_MASK) >> IR_opto_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(IR_opto_INTSTAT) 

    /*******************************************************************************
    * Function Name: IR_opto_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 IR_opto_ClearInterrupt(void) 
    {
        return (IR_opto_INTSTAT & IR_opto_MASK) >> IR_opto_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
