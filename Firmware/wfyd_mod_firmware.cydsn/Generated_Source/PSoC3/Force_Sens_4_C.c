/*******************************************************************************
* File Name: Force_Sens_4_C.c  
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
#include "Force_Sens_4_C.h"


/*******************************************************************************
* Function Name: Force_Sens_4_C_Write
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
void Force_Sens_4_C_Write(uint8 value) 
{
    uint8 staticBits = (Force_Sens_4_C_DR & (uint8)(~Force_Sens_4_C_MASK));
    Force_Sens_4_C_DR = staticBits | ((uint8)(value << Force_Sens_4_C_SHIFT) & Force_Sens_4_C_MASK);
}


/*******************************************************************************
* Function Name: Force_Sens_4_C_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  Force_Sens_4_C_DM_STRONG     Strong Drive 
*  Force_Sens_4_C_DM_OD_HI      Open Drain, Drives High 
*  Force_Sens_4_C_DM_OD_LO      Open Drain, Drives Low 
*  Force_Sens_4_C_DM_RES_UP     Resistive Pull Up 
*  Force_Sens_4_C_DM_RES_DWN    Resistive Pull Down 
*  Force_Sens_4_C_DM_RES_UPDWN  Resistive Pull Up/Down 
*  Force_Sens_4_C_DM_DIG_HIZ    High Impedance Digital 
*  Force_Sens_4_C_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void Force_Sens_4_C_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(Force_Sens_4_C_0, mode);
}


/*******************************************************************************
* Function Name: Force_Sens_4_C_Read
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
*  Macro Force_Sens_4_C_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 Force_Sens_4_C_Read(void) 
{
    return (Force_Sens_4_C_PS & Force_Sens_4_C_MASK) >> Force_Sens_4_C_SHIFT;
}


/*******************************************************************************
* Function Name: Force_Sens_4_C_ReadDataReg
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
uint8 Force_Sens_4_C_ReadDataReg(void) 
{
    return (Force_Sens_4_C_DR & Force_Sens_4_C_MASK) >> Force_Sens_4_C_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(Force_Sens_4_C_INTSTAT) 

    /*******************************************************************************
    * Function Name: Force_Sens_4_C_ClearInterrupt
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
    uint8 Force_Sens_4_C_ClearInterrupt(void) 
    {
        return (Force_Sens_4_C_INTSTAT & Force_Sens_4_C_MASK) >> Force_Sens_4_C_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
