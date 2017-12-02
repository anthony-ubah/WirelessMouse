////////////////////////////////////////////////////////////////////////////////
//	File Name				: cc2500.h
//	Description			: Header file for the cc2500 chip
//	Course					: ECSE 426
//	Authors					: Group 09
//	Date						: December 9th, 2016
////////////////////////////////////////////////////////////////////////////////


#ifndef _CC2500
#define _CC2500

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx.h"
#include "main.h"


//Pin 1: Ground -> GND
//Pin 2: VCC -> 3
//Pin 12: Ground -> GND
//Pin 15: SPI MISO -> PB14	
//Pin 16: SPI SCK -> PB13
//Pin 17: SPI CS -> PB12
//Pin 18: SPI MOSI -> PB15


/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/



/******************************************************************************
Configuration Register Details ? Registers with Preserved Values in SLEEP State
******************************************************************************/

//GDO2 and GDO1 output configuration
#define CC2500_IOCFG2 0x00
#define CC2500_IOCFG1 0x01

/*******************************************************************************
* CC2500_IOCFG0: GDO0 Output Pin Configuration
* 7: TEMP_SENSOR_ENABLE Enable analog temperature sensor. 
		 Write 0 in all other register bits when using temperature sensor. Default is 0.
* 6: GDO0_INV Invert output, i.e. select active low (1) / high (0). Default is 0
* 5-0: GDO0_CFG[5:0] Default is CLK_XOSC/192
********************************************************************************/
#define CC2500_IOCFG0 0x02

/*******************************************************************************
* CC2500_FIFOTHR: RX FIFO and TX FIFO Thresholds
* 7-4: Reserved
* 3-0: Set the threshold for the TX FIFO and RX FIFO. The threshold
			 is exceeded when the number of bytes in the FIFO is equal to
			 or higher than the threshold value. Default is 7.
********************************************************************************/
#define CC2500_FIFOTHR 0x03

//High and low sync bytes
#define CC2500_SYNC1 0x04
#define CC2500_SYNC0 0x05

/*******************************************************************************
* CC2500_PKTLEN: Packet Length
* 7-0: Indicates the packet length when fixed packet length is
			 enabled. If variable length packets are used, this value
			 indicates the maximum length packets allowed.
********************************************************************************/
#define CC2500_PKTLEN 0x06

/*******************************************************************************
* CC2500_PKTCTRL1: Packet Automation Control
* 7-5: Preamble quality estimator threshold. The preamble quality
			 estimator increases an internal counter by one each time a bit is
			 received that is different from the previous bit, and decreases the
			 counter by 8 each time a bit is received that is the same as the
			 last bit.
			 A threshold of 4?PQT for this counter is used to gate sync word
			 detection. When PQT=0 a sync word is always accepted.
* 4: Reserved
* 3: CRC_AUTOFLUSH Enable automatic flush of RX FIFO when CRC is not OK. This
									 requires that only one packet is in the RX FIFO and that packet
									 length is limited to the RX FIFO size.
									 PKTCTRL0.CC2400_EN must be 0 (default) for the CRC
									 autoflush function to work correctly.
* 2: APPEND_STATUS When enabled, two status bytes will be appended to the payload
									 of the packet. The status bytes contain RSSI and LQI values, as
									 well as the CRC OK flag. Default is 1.
* 1-0: ADR_CHK[1:0] Controls address check configuration of received packages.
										Default is no address check (00).
********************************************************************************/
#define CC2500_PKTCTRL1 0x07

/*******************************************************************************
* CC2500_PKTCTRL0: Packet Automation Control
* 7-5: Preamble quality estimator threshold. The preamble quality
			 estimator increases an internal counter by one each time a bit is
			 received that is different from the previous bit, and decreases the
			 counter by 8 each time a bit is received that is the same as the
			 last bit.
			 A threshold of 4?PQT for this counter is used to gate sync word
			 detection. When PQT=0 a sync word is always accepted.
* 4: Reserved
* 3: CRC_AUTOFLUSH Enable automatic flush of RX FIFO when CRC is not OK. This
									 requires that only one packet is in the RX FIFO and that packet
									 length is limited to the RX FIFO size.
									 PKTCTRL0.CC2400_EN must be 0 (default) for the CRC
									 autoflush function to work correctly.
* 2: APPEND_STATUS When enabled, two status bytes will be appended to the payload
									 of the packet. The status bytes contain RSSI and LQI values, as
									 well as the CRC OK flag. Default is 1.
* 1-0: ADR_CHK[1:0] Controls address check configuration of received packages.
										Default is no address check (00).
********************************************************************************/
#define CC2500_PKTCTRL0 0x08

//Device address for packet filtration
#define CC2500_ADDR 0x09

//Channel number
#define CC2500_CHANNR 0x0A

//Frequency Synthesizer Control
#define CC2500_FSCTRL1 0x0B
#define CC2500_FSCTRL0 0x0C

//Frequency control word (High, middle, and low byte)
#define CC2500_FREQ2 0x0D
#define CC2500_FREQ1 0x0E
#define CC2500_FREQ0 0x0F

//Modem configuration
#define CC2500_MDMCFG4 0x10
#define CC2500_MDMCFG3 0x11
#define CC2500_MDMCFG2 0x12
#define CC2500_MDMCFG1 0x13
#define CC2500_MDMCFG0 0x14

//Modem deviation setting
#define CC2500_DEVIATN 0x15

//Main Radio Control State Machine Configuration
#define CC2500_MCSM2 0x16

/*******************************************************************************
* CC2500_MCSM1: Main Radio Control State Machine Configuration
* 7-6: Reserved
* 5-4: CCA_MODE[1:0] Clear channel indication. Default is if RSSI below threshold unless currently
										 receiving a packet.
* 3-2: RXOFF_MODE[1:0] Select what should happen when a packet has been received.
											 Default is IDLE.
* 1-0: TXOFF_MODE[1:0] Select what should happen when a packet has been sent (TX)
											 Default is IDLE.
********************************************************************************/
#define CC2500_MCSM1 0x17

//Main Radio Control State Machine Configuration
#define CC2500_MCSM0 0x18

//Frequency Offset Compensation Configuration
#define CC2500_FOCCFG 0x19

//Bit Synchronization Configuration
#define CC2500_BSCFG 0x1A

//AGC Control
#define CC2500_AGCTRL2 0x1B
#define CC2500_AGCTRL1 0x1C
#define CC2500_AGCTRL0 0x1D

//high byte event0 timeout
#define CC2500_WOREVT1 0x1E

//low byte event0 timeout
#define CC2500_WOREVT0 0x1F

//wake on radio control
#define CC2500_WORCTRL 0x20

//Front End RX Configuration
#define CC2500_FREND1 0x21

//Front End TX configuration
#define CC2500_FREND0 0x22

//Frequency Synthesizer Calibration
#define CC2500_FSCAL3 0x23
#define CC2500_FSCAL2 0x24
#define CC2500_FSCAL1 0x25
#define CC2500_FSCAL0 0x26

//RC Oscillator Configuration
#define CC2500_RCCTRL1 0x27
#define CC2500_RCCTRL0 0x28


/******************************************************************************
Configuration Register Details ? Registers that Lose Programming in SLEEP State
******************************************************************************/

//Frequency Synthesizer Calibration Control
#define CC2500_FSTEST 0x29

//Production Test
#define CC2500_PTEST 0x2A

//AGC Test
#define CC2500_AGCTEST 0x2B

//Various Test Settings
#define CC2500_TEST2 0x2C
#define CC2500_TEST1 0x2D
#define CC2500_TEST0 0x2E

/******************************************************************************
Status Register Details
******************************************************************************/

//Chip part number = 128
#define CC2500_PARTNUM 0x30

//VERSION ? Chip ID = 3
#define CC2500_VERSION 0x31

//Frequency Offset Estimate from Demodulator
#define CC2500_FREQEST 0x32

//Demodulator Estimate for Link Quality
#define CC2500_LQI 0x33

//Received Signal Strength Indication
#define CC2500_RSSI 0x34

//Main Radio Control State Machine State
#define CC2500_MARCSTATE 0x35

//High Byte of WOR Time
#define CC2500_WORTIME1 0x36

//Low Byte of WOR Time
#define CC2500_WORTIME0 0x37

//Current GDOx Status and Packet Status
#define CC2500_PKTSTATUS 0x38

//Current Setting from PLL Calibration Module
#define CC2500_VCO_VC_DAC 0x39

//Underflow and Number of Bytes
#define CC2500_TXBYTES 0x3A

//Overflow and Number of Bytes
#define CC2500_RXBYTES 0x3B

//Last RC Oscillator Calibration Result
#define CC2500_RCCTRL1_STATUS 0x3C
#define CC2500_RCCTRL0_STATUS 0x3D


/******************************************************************************
Command strobes
******************************************************************************/

//Reset chip
#define CC2500_SRES 0x30

//Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA):
//Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC2500_SFSTXON 0x31

//Turn off crystal oscillator
#define CC2500_SXOFF 0x32

//Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without
//setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
#define CC2500_SCAL 0x33

//Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
#define CC2500_SRX 0x34

//In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
//If in RX state and CCA is enabled: Only go to TX if channel is clear
#define CC2500_STX 0x35

//Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
#define CC2500_SIDLE 0x36

//Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if
//WORCTRL.RC_PD=0.
#define CC2500_SWOR 0x38

//Enter power down mode when CSn goes high.
#define CC2500_SPWD 0x39

//Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
#define CC2500_SFRX 0x3A

//Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
#define CC2500_SFTX 0x3B

//Reset real time clock to Event1 value.
#define CC2500_SWORRST 0x3C

//No operation. May be used to get access to the chip status byte.
#define CC2500_SNOP 0x3D
#define CC2500_PATABLE 0x3E

//RX and TX FIFO
#define CC2500_FIFO 0x3F

/**
  * @brief  CC2500 Register settings
  */
#define VAL_CC2500_IOCFG2 	0x29 		
#define VAL_CC2500_IOCFG0 	0x06	

#define VAL_CC2500_FIFOTHR 	0x07

#define VAL_CC2500_PKTLEN 	0x01

#define VAL_CC2500_PKTCTRL1 0x08	
#define VAL_CC2500_PKTCTRL0 0x04	

#define VAL_CC2500_ADDR 	0x00	

#define VAL_CC2500_CHANNR 	0x00

#define VAL_CC2500_FSCTRL1 	0x0C	
#define VAL_CC2500_FSCTRL0 	0x00	

// freq set to 2.433 GHz, note f_xosc = 26MHz
//(2400 +9*8)MHz = 72.0024 Mhz

#define VAL_CC2500_FREQ2 	0x5D	
#define VAL_CC2500_FREQ1 	0x94	
#define VAL_CC2500_FREQ0 	0x02	

#define VAL_CC2500_MDMCFG4 	0x0E
#define VAL_CC2500_MDMCFG3 	0x3B
#define VAL_CC2500_MDMCFG2 	0x73	
#define VAL_CC2500_MDMCFG1 	0x42	
#define VAL_CC2500_MDMCFG0 	0xF8	

#define VAL_CC2500_DEVIATN 	0x00	


#define VAL_CC2500_MCSM1 	0x02	
#define VAL_CC2500_MCSM0 	0x18	

#define VAL_CC2500_FOCCFG 	0x1D	
#define VAL_CC2500_BSCFG 	0x1C	

#define VAL_CC2500_AGCTRL2 	0xC7	 
#define VAL_CC2500_AGCTRL1 	0x40	
#define VAL_CC2500_AGCTRL0 	0xB0

#define VAL_CC2500_FREND1 	0xB6	 
#define VAL_CC2500_FREND0 	0x10	

#define VAL_CC2500_FSCAL3 	0xEA	 
#define VAL_CC2500_FSCAL2 	0x0A	
#define VAL_CC2500_FSCAL1 	0x00	 
#define VAL_CC2500_FSCAL0 	0x19	

#define VAL_CC2500_FSTEST 	0x59

#define VAL_CC2500_TEST2 	0x88	
#define VAL_CC2500_TEST1 	0x31	
#define VAL_CC2500_TEST0 	0x0B

//pin configuration
#define CC2500_SPI                       SPI2
#define CC2500_SPI_CLK                   RCC_APB2Periph_SPI2
        
#define CC2500_SPI_SCK_PIN               GPIO_PIN_13                  /* PB13 */
#define CC2500_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
        
#define CC2500_SPI_MISO_PIN              GPIO_PIN_14                  /* PB14 */
#define CC2500_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
        
#define CC2500_SPI_MOSI_PIN              GPIO_PIN_15                  /* PB15 */
#define CC2500_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOB */
        
#define CC2500_SPI_CS_PIN                GPIO_PIN_12                  /* PB12 */
#define CC2500_SPI_CS_GPIO_PORT          GPIOB                       /* GPIOB */

       
//#define CC2500_SPI_INT1_PIN              GPIO_PIN_0                  /* PE00 */
//#define CC2500_SPI_INT1_GPIO_PORT        GPIOE                       /* GPIOE */
//#define CC2500_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOE
//#define CC2500_SPI_INT1_EXTI_LINE        EXTI_Line0
//#define CC2500_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
//#define CC2500_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
//#define CC2500_SPI_INT1_EXTI_IRQn        EXTI0_IRQn
//       
//#define CC2500_SPI_INT2_PIN              GPIO_PIN_1                  /* PE01 */
//#define CC2500_SPI_INT2_GPIO_PORT        GPIOE                       /* GPIOE */
//#define CC2500_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOE
//#define CC2500_SPI_INT2_EXTI_LINE        EXTI_Line1
//#define CC2500_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
//#define CC2500_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
//#define CC2500_SPI_INT2_EXTI_IRQn        EXTI1_IRQn

#define CC2500_TIMEOUT_THRESH ((uint32_t)0x1000)
#define CC2500_CS_LOW()  HAL_GPIO_WritePin(CC2500_SPI_CS_GPIO_PORT, CC2500_SPI_CS_PIN, GPIO_PIN_RESET)
#define CC2500_CS_HIGH() HAL_GPIO_WritePin(CC2500_SPI_CS_GPIO_PORT, CC2500_SPI_CS_PIN, GPIO_PIN_SET)

#define SIZE_OF_FIFO 64
#define RX_OVERFLOW 6

uint8_t cc2500_SPI_ReceiveData(SPI_HandleTypeDef *hspi);
void cc2500_SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t byte);
void cc2500_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void cc2500_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
uint8_t cc2500_SendByte(uint8_t byte);
void cc2500_Send_Command_Strobe(uint8_t* pBuffer, uint8_t ReadAddr);
void cc2500_Read_Status_Register(uint8_t* pBuffer, uint8_t ReadAddr);
void cc2500_One_Byte_Read(uint8_t* pBuffer, uint8_t ReadAddr);
void cc2500_Init(void);
void cc2500_Chipset_Init(void);
void cc2500_SPI_Init(void);
void cc2500_config_registers(void);
uint8_t cc2500_Receive_Data(uint8_t* output_array);
void cc2500_start_up_procedure(void);
void cc2500_SendPacket(uint8_t num);
void cc2500_ReadPacket(uint8_t* packet);
void cc2500_tx_config(void);

#endif