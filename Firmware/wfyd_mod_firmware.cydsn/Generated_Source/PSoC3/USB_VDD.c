/*******************************************************************************
* File Name: USB_VDD.c  
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
#include "USB_VDD.h"


/*******************************************************************************
* Function Name: USB_VDD_Write
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
void USB_VDD_Write(uint8 value) 
{
    uint8 staticBits = (USB_VDD_DR & (uint8)(~USB_VDD_MASK));
    USB_VDD_DR = staticBits | ((uint8)(value << USB_VDD_SHIFT) & USB_VDD_MASK);
}


/*******************************************************************************
* Function Name: USB_VDD_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  USB_VDD_DM_STRONG     Strong Drive 
*  USB_VDD_DM_OD_HI      Open Drain, Drives High 
*  USB_VDD_DM_OD_LO      Open Drain, Drives Low 
*  USB_VDD_DM_RES_UP     Resistive Pull Up 
*  USB_VDD_DM_RES_DWN    Resistive Pull Down 
*  USB_VDD_DM_RES_UPDWN  Resistive Pull Up/Down 
*  USB_VDD_DM_DIG_HIZ    High Impedance Digital 
*  USB_VDD_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void USB_VDD_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(USB_VDD_0, mode);
}


/*******************************************************************************
* Function Name: USB_VDD_Read
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
*  Macro USB_VDD_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 USB_VDD_Read(void) 
{
    return (USB_VDD_PS & USB_VDD_MASK) >> USB_VDD_SHIFT;
}


/*******************************************************************************
* Function Name: USB_VDD_ReadDataReg
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
uint8 USB_VDD_ReadDataReg(void) 
{
    return (USB_VDD_DR & USB_VDD_MASK) >> USB_VDD_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(USB_VDD_INTSTAT) 

    /*******************************************************************************
    * Function Name: USB_VDD_ClearInterrupt
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
    uint8 USB_VDD_ClearInterrupt(void) 
    {
        return (USB_VDD_INTSTAT & USB_VDD_MASK) >> USB_VDD_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
