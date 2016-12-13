/*******************************************************************************
* File Name: MOTOR_EN.c  
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
#include "MOTOR_EN.h"


/*******************************************************************************
* Function Name: MOTOR_EN_Write
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
void MOTOR_EN_Write(uint8 value) 
{
    uint8 staticBits = (MOTOR_EN_DR & (uint8)(~MOTOR_EN_MASK));
    MOTOR_EN_DR = staticBits | ((uint8)(value << MOTOR_EN_SHIFT) & MOTOR_EN_MASK);
}


/*******************************************************************************
* Function Name: MOTOR_EN_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  MOTOR_EN_DM_STRONG     Strong Drive 
*  MOTOR_EN_DM_OD_HI      Open Drain, Drives High 
*  MOTOR_EN_DM_OD_LO      Open Drain, Drives Low 
*  MOTOR_EN_DM_RES_UP     Resistive Pull Up 
*  MOTOR_EN_DM_RES_DWN    Resistive Pull Down 
*  MOTOR_EN_DM_RES_UPDWN  Resistive Pull Up/Down 
*  MOTOR_EN_DM_DIG_HIZ    High Impedance Digital 
*  MOTOR_EN_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void MOTOR_EN_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(MOTOR_EN_0, mode);
	CyPins_SetPinDriveMode(MOTOR_EN_1, mode);
}


/*******************************************************************************
* Function Name: MOTOR_EN_Read
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
*  Macro MOTOR_EN_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 MOTOR_EN_Read(void) 
{
    return (MOTOR_EN_PS & MOTOR_EN_MASK) >> MOTOR_EN_SHIFT;
}


/*******************************************************************************
* Function Name: MOTOR_EN_ReadDataReg
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
uint8 MOTOR_EN_ReadDataReg(void) 
{
    return (MOTOR_EN_DR & MOTOR_EN_MASK) >> MOTOR_EN_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(MOTOR_EN_INTSTAT) 

    /*******************************************************************************
    * Function Name: MOTOR_EN_ClearInterrupt
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
    uint8 MOTOR_EN_ClearInterrupt(void) 
    {
        return (MOTOR_EN_INTSTAT & MOTOR_EN_MASK) >> MOTOR_EN_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
