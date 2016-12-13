/*******************************************************************************
* File Name: VOLTAGE_SENSE.c  
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
#include "VOLTAGE_SENSE.h"


/*******************************************************************************
* Function Name: VOLTAGE_SENSE_Write
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
void VOLTAGE_SENSE_Write(uint8 value) 
{
    uint8 staticBits = (VOLTAGE_SENSE_DR & (uint8)(~VOLTAGE_SENSE_MASK));
    VOLTAGE_SENSE_DR = staticBits | ((uint8)(value << VOLTAGE_SENSE_SHIFT) & VOLTAGE_SENSE_MASK);
}


/*******************************************************************************
* Function Name: VOLTAGE_SENSE_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  VOLTAGE_SENSE_DM_STRONG     Strong Drive 
*  VOLTAGE_SENSE_DM_OD_HI      Open Drain, Drives High 
*  VOLTAGE_SENSE_DM_OD_LO      Open Drain, Drives Low 
*  VOLTAGE_SENSE_DM_RES_UP     Resistive Pull Up 
*  VOLTAGE_SENSE_DM_RES_DWN    Resistive Pull Down 
*  VOLTAGE_SENSE_DM_RES_UPDWN  Resistive Pull Up/Down 
*  VOLTAGE_SENSE_DM_DIG_HIZ    High Impedance Digital 
*  VOLTAGE_SENSE_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void VOLTAGE_SENSE_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(VOLTAGE_SENSE_0, mode);
}


/*******************************************************************************
* Function Name: VOLTAGE_SENSE_Read
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
*  Macro VOLTAGE_SENSE_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 VOLTAGE_SENSE_Read(void) 
{
    return (VOLTAGE_SENSE_PS & VOLTAGE_SENSE_MASK) >> VOLTAGE_SENSE_SHIFT;
}


/*******************************************************************************
* Function Name: VOLTAGE_SENSE_ReadDataReg
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
uint8 VOLTAGE_SENSE_ReadDataReg(void) 
{
    return (VOLTAGE_SENSE_DR & VOLTAGE_SENSE_MASK) >> VOLTAGE_SENSE_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(VOLTAGE_SENSE_INTSTAT) 

    /*******************************************************************************
    * Function Name: VOLTAGE_SENSE_ClearInterrupt
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
    uint8 VOLTAGE_SENSE_ClearInterrupt(void) 
    {
        return (VOLTAGE_SENSE_INTSTAT & VOLTAGE_SENSE_MASK) >> VOLTAGE_SENSE_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
