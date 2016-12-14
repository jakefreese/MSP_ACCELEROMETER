//  Interfacing ADXL345 accelerometer with MSP430G2553 with I2C communication
//  and printing restuls to serial port using UART.
//
//                                /|\  /|\
//               ADXL345          10k  10k     MSP430G2xx3
//                slave            |    |        master
//             -----------------   |    |  -----------------
//            |              SDA|<-|---+->|P1.7/UCB0SDA  XIN|-
//            |                 |  |      |                 |
//            |                 |  |      |             XOUT|-
//            |              SCL|<-+----->|P1.6/UCB0SCL     |
//            |                 |         |                 |
//
//  For Sparkfun ADXL345,
//    * connect SDO to ground
//    * connect CS to VCC
//
//  Original I2C code by :
//  Prof. Ravi Butani
//  Marwadi Education Foundation, Rajkot GUJARAT-INDIA
//  ravi.butani@marwadieducation.edu.in
//  <a href="https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/260094">e2e.ti.com/.../260094</a>
//
//  Original UART code by :
//  Benn Thomsen
//  <a href="https://bennthomsen.wordpress.com/engineering-toolbox/ti-msp430-launchpad/msp430g2553-hardware-uart/">bennthomsen.wordpress.com/.../</a>
//  
//
//  Modified By :
//  Phitchaya Mangpo Phothilimthana
//  mangpo@eecs.berkeley.edu

//******************************************************************************
#include <msp430g2553.h>

#define NUM_BYTES_TX 2  
#define NUM_BYTES_RX 6
#define ADXL_345     0x53

int RXByteCtr;
volatile unsigned char RxBuffer[6];         // Allocate 6 byte of RAM
unsigned char *PRxData;                     // Pointer to RX data
unsigned char TXByteCtr, RX = 0;
unsigned char MSData[2];

// Functions for I2C
void Setup_TX(unsigned char);
void Setup_RX(unsigned char);
void Transmit(unsigned char,unsigned char);
void TransmitOne(unsigned char);
void Receive(void);

// Function for UART: printing output to serial port
void Setup_UART();
void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength);
void UARTSendInt(unsigned int x);

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;  // Stop WDT

  // LED
  P1DIR |= BIT0; // P1.0 = red LED
  P1OUT |= BIT0; // P1.0 = red LED

  // UART
  BCSCTL1 = CALBC1_1MHZ; // Set DCO to 1MHz
  DCOCTL = CALDCO_1MHZ; // Set DCO to 1MHz

  // Configure hardware UART
  P1SEL |= BIT1 + BIT2 ;  // P1.1 = RXD, P1.2=TXD
  P1SEL2 |= BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD

  // ADXL345
  P1SEL  |= BIT6 + BIT7;  // Assign I2C pins to USCI_B0, P1.6 = SCL, P1.7 = SDA
  P1SEL2 |= BIT6 + BIT7;  // Assign I2C pins to USCI_B0, P1.6 = SCL, P1.7 = SDA

  // Init sequence for ADXL345
  Setup_TX(ADXL_345);
  Transmit(0x2D,0x00);                    
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

  Setup_TX(ADXL_345);
  Transmit(0x2D,0x10);
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

  Setup_TX(ADXL_345);
  Transmit(0x2D,0x08);
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  
  // Un-comment next block to change range of ADXL345
  /*
    Setup_TX(ADXL_345);
    RPT_Flag = 1;
    Transmit(0x31,0x01);                // Range Select at add 0x31 write 0x00 for 2g(default)/ 0x01 for 4g/ 0x02 for 8g/ 0x03 for 16g
    while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
  */

  while(1){
    // Transmit process
    Setup_TX(ADXL_345);
    TransmitOne(0x32);                  // Request Data from ADXL345 in 2g Range 10Bit resolution
    while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent

    // Receive process
    Setup_RX(ADXL_345);
    Receive();
    while (UCB0CTL1 & UCTXSTP);          // Ensure stop condition got sent

    int x = (((int)RxBuffer[1]) << 8) | RxBuffer[0];
    int y = (((int)RxBuffer[3]) << 8) | RxBuffer[2];
    int z = (((int)RxBuffer[5]) << 8) | RxBuffer[4];

    // Now we have x,y,z reading.
    // Below red LED is on, if x or y angle is more then 45 or less then -45 degree.
    if ((x > 128) || (y > 128) || (x < -128) || (y < -128)) {
      P1OUT |= BIT0; // red LED on
    }
    else {
      P1OUT &= ~BIT0; // red LED off
    }

    // Print x,y,z to serial port in hew.
    Setup_UART();
    UARTSendArray("sample\n", 7);
    UARTSendInt(x);
    UARTSendInt(y);
    UARTSendInt(z);
    __delay_cycles(1000000);  // delay 1 sec
  }
}

//-------------------------------------------------------------------------------
// I2C
//-------------------------------------------------------------------------------
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
  
  if(RX == 1){                              // Master Recieve?
    RXByteCtr--;                            // Decrement RX byte counter
    if (RXByteCtr)
      {
        *PRxData++ = UCB0RXBUF;             // Move RX data to address PRxData
      }
    else
      {
        UCB0CTL1 |= UCTXSTP;                // No Repeated Start: stop condition
        *PRxData++ = UCB0RXBUF;             // Move final RX data to PRxData
        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
      }}
  else{                                     // Master Transmit
    if (TXByteCtr)                          // Check TX byte counter
      {
        TXByteCtr--;                        // Decrement TX byte counter
        UCB0TXBUF = MSData[TXByteCtr];      // Load TX buffer
      }
    else
      {
          UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
          IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
      }
  }
}

void Setup_TX(unsigned char Dev_ID){
  _DINT();
  RX = 0;
  IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt (UART)
  IE2 &= ~UCB0RXIE; // Disable USCI_B0 RX interrupt (I2C)
  while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent// Disable RX interrupt
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  IE2 |= UCB0TXIE;                          // Enable TX interrupt
}

void Setup_RX(unsigned char Dev_ID){
  _DINT();
  RX = 1;
  IE2 &= ~UCA0RXIE; // Disable USCI_A0 RX interrupt (UART)
  IE2 &= ~UCB0TXIE; // Disable USCI_B0 TX interrupt (I2C)
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB0BR0 = 12;                             // fSCL = SMCLK/12 = ~100kHz
  UCB0BR1 = 0;
  UCB0I2CSA = Dev_ID;                       // Slave Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  IE2 |= UCB0RXIE;                          // Enable RX interrupt
}

void Transmit(unsigned char Reg_ADD,unsigned char Reg_DAT){
  MSData[1]= Reg_ADD;
  MSData[0]= Reg_DAT;
  TXByteCtr = NUM_BYTES_TX;               // Load TX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void TransmitOne(unsigned char Reg_ADD){
  MSData[0]= Reg_ADD;
  TXByteCtr = 1;                          // Load TX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void Receive(void){
  PRxData = (unsigned char *)RxBuffer;    // Start of RX buffer
  RXByteCtr = NUM_BYTES_RX;               // Load RX byte counter
  while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
  UCB0CTL1 |= UCTXSTT;                    // I2C start condition
  __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

//-------------------------------------------------------------------------------
// UART
//-------------------------------------------------------------------------------

void Setup_UART() {
  _DINT();
  IE2 &= ~UCB0RXIE; // Disable USCI_B0 RX interrupt (I2C)
  IE2 &= ~UCB0TXIE; // Disable USCI_B0 TX interrupt (I2C)
  UCA0CTL1 |= UCSSEL_2; // Use SMCLK
  UCA0BR0 = 104;        // Set baud rate to 9600 with 1MHz clock (Data Sheet 15.3.13)
  UCA0BR1 = 0;          // Set baud rate to 9600 with 1MHz clock
  UCA0MCTL = UCBRS0;    // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
  IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt
}

// Print an array of char
void UARTSendArray(unsigned char *TxArray, unsigned char ArrayLength){

  while(ArrayLength--){         // Loop until StringLength == 0 and post decrement
    while(!(IFG2 & UCA0TXIFG)); // Wait for TX buffer to be ready for new data
    UCA0TXBUF = *TxArray;       // Write the character at the location specified py the pointer
    TxArray++;                  //Increment the TxString pointer to point to the next character
  }
  IFG2 &= ~UCA0TXIFG;           // Clear USCI_A0 int flag
}

// Print int in hex
void UARTSendInt(unsigned int x){
  unsigned char buff[10];
  unsigned char data[10];
  unsigned char index = 0, i = 0;

  while(x > 0) {
    unsigned char val = x % 16;
    if(val < 10)
      buff[index] = 48+val;
    else
      buff[index] = 97+val-10;
    index++;
    x /= 16;
  }
  buff[index] = '\n';

  while(index > 0) {
    index--;
    data[i] = buff[index];
    i++;
  }

  if(i==0) {
    data[0] = '0';
    i++;
  }
  data[i] = '\n';
  UARTSendArray(data, i+1);
}
