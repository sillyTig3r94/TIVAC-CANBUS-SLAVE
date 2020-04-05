#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/fpu.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "grlib/grlib.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"

volatile uint32_t g_ui32RXMsgCount = 0;
volatile uint32_t g_ui32TXMsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// A global to keep track of the error flags that have been thrown so they may
// be processed. This is necessary because reading the error register clears
// the flags, so it is necessary to save them somewhere for processing.
//
//*****************************************************************************
volatile uint32_t g_ui32ErrFlag = 0;

//*****************************************************************************
//
// CAN message Objects for data being sent / received
//
//*****************************************************************************
tCANMsgObject g_sCAN0RxMessage;
tCANMsgObject g_sCAN0TxMessage;

//*****************************************************************************
//
// Message Identifiers and Objects
// RXID is set to 0 so all messages are received
//
//*****************************************************************************
#define CAN0RXID                0
#define RXOBJECT                1
#define CAN0TXID                2
#define TXOBJECT                2

//*****************************************************************************
//
// Variables to hold character being sent / received
//
//*****************************************************************************
uint8_t g_ui8TXMsgData;
uint8_t g_ui8RXMsgData[8];

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// CAN 0 Interrupt Handler. It checks for the cause of the interrupt, and
// maintains a count of all messages that have been transmitted / received
//
//*****************************************************************************
void
CAN0IntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    // CAN_INT_STS_CAUSE register values
    // 0x0000        = No Interrupt Pending
    // 0x0001-0x0020 = Number of message object that caused the interrupt
    // 0x8000        = Status interrupt
    // all other numbers are reserved and have no meaning in this system
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If this was a status interrupt acknowledge it by reading the CAN
    // controller status register.
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors. Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Add ERROR flags to list of current errors. To be handled
        // later, because it would take too much time here in the
        // interrupt.
        //
        g_ui32ErrFlag |= ui32Status;
    }

    //
    // Check if the cause is message object RXOBJECT, which we are using
    // for receiving messages.
    //
    else if(ui32Status == RXOBJECT)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object RXOBJECT, and the message reception is complete.
        // Clear the message object interrupt.
        //
        CANIntClear(CAN0_BASE, RXOBJECT);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32RXMsgCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = true;

        //
        // Since a message was received, clear any error flags.
        // This is done because before the message is received it triggers
        // a Status Interrupt for RX complete. by clearing the flag here we
        // prevent unnecessary error handling from happeneing
        //
        g_ui32ErrFlag = 0;
    }

    //
    // Check if the cause is message object TXOBJECT, which we are using
    // for transmitting messages.
    //
    else if(ui32Status == TXOBJECT)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object TXOBJECT, and the message reception is complete.
        // Clear the message object interrupt.
        //
        CANIntClear(CAN0_BASE, TXOBJECT);

        //
        // Increment a counter to keep track of how many messages have been
        // transmitted. In a real application this could be used to set
        // flags to indicate when a message is transmitted.
        //
        g_ui32TXMsgCount++;

        //
        // Since a message was transmitted, clear any error flags.
        // This is done because before the message is transmitted it triggers
        // a Status Interrupt for TX complete. by clearing the flag here we
        // prevent unnecessary error handling from happeneing
        //
        g_ui32ErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

//============================================================================//
//-----------------------------UART INITIALIZATION----------------------------//
//============================================================================//
void ConfigureUART(void)
{
    /*
     * UART0, hardware interrupt = 21
     * Baud rate = 115200
     * PA0 - RX, PA1 - TX
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // Enable UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART clock source.
    UARTStdioConfig(0, 115200, 16000000); // set up baud rate speed
}

//============================================================================//
//------------------------------CAN INITIALIZATION----------------------------//
//============================================================================//
void InitCAN0(void)
{
    /*
     * Setup CAN0 to both send and receive at 500kps, interrupt enable
     * Pin Configure: PE4 - RX and PE5 - TX
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable Peripheral on Port E
    GPIOPinConfigure(GPIO_PE4_CAN0RX); // Configure PE4 as CAN - RX
    GPIOPinConfigure(GPIO_PE5_CAN0TX); // Configure PE5 as CAN - TX
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0); // Enable Peripheral - CAN 0 Module
    CANInit(CAN0_BASE); // Initialize the CAN controller
    //
    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.
    //
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
    CANIntRegister(CAN0_BASE, CAN0IntHandler);
    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.
    //
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0); // Enable the CAN interrupt on the processor (NVIC).
    CANEnable(CAN0_BASE); // Enable the CAN for operation.

    // Initialize a message object to be used for receiving CAN messages with
    // any CAN ID.  In order to receive any CAN ID, the ID and mask must both
    // be set to 0, and the ID filter enabled.
    //
    g_sCAN0RxMessage.ui32MsgID = CAN0RXID;
    g_sCAN0RxMessage.ui32MsgIDMask = 0;
    g_sCAN0RxMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    g_sCAN0RxMessage.ui32MsgLen = sizeof(g_ui8RXMsgData);

    //
    // Now load the message object into the CAN peripheral.  Once loaded the
    // CAN will receive any message on the bus, and an interrupt will occur.
    // Use message object RXOBJECT for receiving messages (this is not the
    //same as the CAN ID which can be any value in this example).
    //
    CANMessageSet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, MSG_OBJ_TYPE_RX);

    //
    // Initialize the message object that will be used for sending CAN
    // messages.  The message will be 1 bytes that will contain the character
    // received from the other controller. Initially it will be set to 0.
    //
    g_ui8TXMsgData = 0;
    g_sCAN0TxMessage.ui32MsgID = CAN0TXID;
    g_sCAN0TxMessage.ui32MsgIDMask = 0;
    g_sCAN0TxMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    g_sCAN0TxMessage.ui32MsgLen = sizeof(g_ui8TXMsgData);
    g_sCAN0TxMessage.pui8MsgData = (uint8_t *)&g_ui8TXMsgData;
}

/******************************************************************************
 * Can ERROR handling. When a message is received if there is an error it is
 * saved to g_ui32ErrFlag, the Error Flag Set. Below the flags are checked
 * and cleared. It is left up to the user to add handling functionality if so
 * desired.
 ******************************************************************************/
void CANErrorHandler(void)
{

    if(g_ui32ErrFlag & CAN_STATUS_BUS_OFF) // CAN controller has entered a Bus Off state.
    {
        // Handle Error Condition here
        UARTprintf("\nERROR: CAN_STATUS_BUS_OFF \n");

        g_ui32ErrFlag &= ~(CAN_STATUS_BUS_OFF);  // Clear CAN_STATUS_BUS_OFF Flag

    }

    if(g_ui32ErrFlag & CAN_STATUS_EWARN) // CAN controller error level has reached warning level.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_EWARN); // Clear CAN_STATUS_EWARN Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_EPASS) // CAN controller error level has reached error passive level.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_EPASS); // Clear CAN_STATUS_EPASS Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_RXOK)  // A message was received successfully since the last read of this status.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_RXOK); // Clear CAN_STATUS_RXOK Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_TXOK)  // A message was transmitted successfully since the last read of this status.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_TXOK); // Clear CAN_STATUS_TXOK Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_MSK) // This is the mask for the last error code field.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MSK); // Clear CAN_STATUS_LEC_MSK Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_STUFF) // A bit stuffing error has occurred.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_STUFF); // Clear CAN_STATUS_LEC_STUFF Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_FORM) // A formatting error has occurred.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_FORM); // Clear CAN_STATUS_LEC_FORM Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_ACK) // An acknowledge error has occurred.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_ACK); // Clear CAN_STATUS_LEC_ACK Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT1) // The bus remained a bit level of 1 for longer than is allowed.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT1); // Clear CAN_STATUS_LEC_BIT1 Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_BIT0) // The bus remained a bit level of 0 for longer than is allowed.
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_BIT0); // Clear CAN_STATUS_LEC_BIT0 Flag
    }

    if(g_ui32ErrFlag & CAN_STATUS_LEC_CRC) // A CRC error has occurred.
    {
        // Handle Error Condition here


        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_CRC); // Clear CAN_STATUS_LEC_CRC Flag
    }


    if(g_ui32ErrFlag & CAN_STATUS_LEC_MASK) // This is the mask for the CAN Last Error Code (LEC).
    {
        // Handle Error Condition here

        g_ui32ErrFlag &= ~(CAN_STATUS_LEC_MASK); // Clear CAN_STATUS_LEC_MASK Flag
    }

    /*
     * If there are any bits still set in g_ui32ErrFlag then something unhandled
     * has happened. Print the value of g_ui32ErrFlag.
     */
    if(g_ui32ErrFlag !=0)
        UARTprintf("\nUnhandled ERROR: %x \n",g_ui32ErrFlag); //Print out which error handle
}

//============================================================================//
//--------------------------------MAIN PROGRAM--------------------------------//
//============================================================================//
int main(void)
{
    uint32_t message_id = 0;

    FPULazyStackingEnable(); // Enable lazy stacking for interrupt handlers.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN); // Set the clocking to run directly from the crystal.
    ConfigureUART(); // Initialize the UART
    InitCAN0(); // Initialize CAN0
    UARTprintf("\nProject: Can bus slave... \n\n\r"); // Print welcome message
    while(1)
    {
        if(g_bRXFlag)
        {
            g_sCAN0RxMessage.pui8MsgData = (uint8_t *) &g_ui8RXMsgData;

            CANMessageGet(CAN0_BASE, RXOBJECT, &g_sCAN0RxMessage, 0);    // Read the message from the CAN.

            g_bRXFlag = 0; //Clear the pending message flag so that the interrupt handler can set it again when the next message arrives.

            if(g_sCAN0RxMessage.ui32Flags & MSG_OBJ_DATA_LOST)  //Checking for lost message
                UARTprintf("\nCAN message loss detected\n");

            UARTprintf("Message ID: 0x%03x Message Data:",g_sCAN0RxMessage.ui32MsgID);
            UARTprintf("0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x \n\r"
                    ,g_ui8RXMsgData[0],g_ui8RXMsgData[1],g_ui8RXMsgData[2],g_ui8RXMsgData[3]
                    ,g_ui8RXMsgData[4],g_ui8RXMsgData[5],g_ui8RXMsgData[6],g_ui8RXMsgData[7] ); // Print the received character to the UART terminal
        }
    }
}
