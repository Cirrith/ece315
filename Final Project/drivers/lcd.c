#include "lcd.h"
#include "../include/spi.h"
#include "gpioPort.h"
#include "fonts.h"

//*****************************************************************************
// Initializes the pins needed to communicate with the LCD and issues the 
// initialization sequence found in the LCD datasheet via the SPI interface
//*****************************************************************************
bool lcdSuperInit(void){
	GPIOA_Type  *gpioResetPtr;
  GPIOA_Type  *gpioCmdPtr;
	uint8_t tx_data;
  uint8_t rx_data;
	
	  if( LCD_SPI_BASE == 0 || GPIOD_BASE == 0)
  {
    return false;
  }
	
	gpioResetPtr  = (GPIOA_Type *)GPIOD_BASE;
  gpioCmdPtr    = (GPIOA_Type *)GPIOD_BASE;
	
	//Take the LCD out of reset
  gpioResetPtr->DATA |= LCD_RST_N_PIN;
 
  //Enter Command Mode
  gpioCmdPtr->DATA &= ~LCD_CD_PIN;
 
 //Set Scroll Line
  tx_data = 0x40;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set SEG Directions
  tx_data = 0xA1;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set COM direction
  tx_data = 0xC0;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set All Pixel on
  tx_data = 0xA4;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set Inverse Display
  tx_data = 0xA6;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //et LCD Bias Ratio
  tx_data = 0xA2;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set Power Control
  tx_data = 0x2F;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set VLCD Resistor Ratio
  tx_data = 0x27;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set Electronic Volume
  tx_data = 0x81;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
  tx_data = 0x10;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Set Adv Program Control
  tx_data = 0xFA;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
  tx_data = 0x90;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
 //Set Display Enable
  tx_data = 0xAF;
  spiTx(LCD_SPI_BASE,&tx_data, 1, &rx_data);
 
  //Exit Command Mode
  gpioCmdPtr->DATA |= LCD_CD_PIN;
 
  return true;
}

void ece315_lcdInit(void)
{	
	gpio_enable_port(LCD_GPIO_BASE);
  // Configure SPI CLK
  gpio_config_digital_enable(LCD_GPIO_BASE, LCD_CLK_PIN);
  gpio_config_alternate_function(LCD_GPIO_BASE, LCD_CLK_PIN);
  gpio_config_port_control(LCD_GPIO_BASE, LCD_CLK_PIN_PCTL);
    
  // Configure SPI CS
  gpio_config_digital_enable(LCD_GPIO_BASE, LCD_CS_PIN);
  gpio_config_alternate_function(LCD_GPIO_BASE, LCD_CS_PIN);
  gpio_config_port_control(LCD_GPIO_BASE, LCD_CS_PIN_PCTL);

  // Configure SPI MOSI
  gpio_config_digital_enable(LCD_GPIO_BASE, LCD_MOSI_PIN);
  gpio_config_alternate_function(LCD_GPIO_BASE, LCD_MOSI_PIN);
  gpio_config_port_control(LCD_GPIO_BASE, LCD_MOSI_PIN_PCTL);
  
  // Configure CD
  gpio_config_digital_enable(GPIO_LCD_CD_BASE,LCD_CD_PIN);
  gpio_config_enable_output(GPIO_LCD_CD_BASE,LCD_CD_PIN);
  
  // Configure RST_N
  gpio_config_digital_enable(GPIO_LCD_RST_N_BASE, LCD_RST_N_PIN);
  gpio_config_enable_output(GPIO_LCD_RST_N_BASE, LCD_RST_N_PIN);
  
  initialize_spi( LCD_SPI_BASE, 3);
  
  
  // Bring the LCD out of reset
	
  // Use spiTx() from the ece315 driver library to issue the sequence of 
  // commands in the LCD data sheet to initialize the LCD.  
  lcdSuperInit();
  
}

//*****************************************************************************
// The LCD can be put into command mode by writing a 0 to the CMD pin.  
// In command mode, you can set the active page or column.
// 
// ADD CODE
//*****************************************************************************
__INLINE static void lcd_assert_cmd_mode(void)
{
	GPIOD->DATA &= ~(LCD_CD_PIN);
}

//*****************************************************************************
// Turn OFF command mode by writeing a 1 to the CMD pin
//
// When the LCD is not in command command mode, any data that is written over
// the SPI interface is treated as data to turn ON/OFF pixels at the location
// indicated by the currently active page and column.
// 
// ADD CODE
//*****************************************************************************
__INLINE static void lcd_deassert_cmd_mode(void)
{
	GPIOD->DATA |= (LCD_CD_PIN);
}

//****************************************************************************
// Sets the currently active page
//*****************************************************************************
  void ece315_lcdSetPage(uint8_t   page)
  {
		uint8_t tx_data;
    uint8_t rx_data;
		lcd_assert_cmd_mode();
    tx_data = (0xB0 | (page & 0x0F));
		spiTx(LCD_SPI_BASE, &tx_data, 1, &rx_data);
		lcd_deassert_cmd_mode();
  }
  
//*****************************************************************************
// Sets the currently active column
//*****************************************************************************
void ece315_lcdSetColumn(uint8_t   column)
{
    uint8_t tx_data;
    uint8_t rx_data;
    
    
    //Enter Command Mode
		lcd_assert_cmd_mode();
    
    // Set the active column LSB
		tx_data = (0x00 | (column & 0x0F));
		spiTx(LCD_SPI_BASE, &tx_data, 1, &rx_data);
                    
    // Set the active column MSB
		tx_data = (0x10 | ((column & 0xF0) >> 4));
		spiTx(LCD_SPI_BASE, &tx_data, 1, &rx_data);
    
    //Exit Command Mode
		lcd_deassert_cmd_mode();
}
  
//*****************************************************************************
// Writes 8-bits of data to the current column of the LCD
//*****************************************************************************
  void ece315_lcdWriteData(uint8_t   data)
  {
      uint8_t rx_data;
    
    //Exit Command Mode
		lcd_deassert_cmd_mode();

    // Send the data
		spiTx(LCD_SPI_BASE, &data, 1, &rx_data);
  }
  
//*****************************************************************************
// Erases the LCD screen.
//*****************************************************************************
 void ece315_lcdClear(void)
 {
   uint8_t page;
  uint8_t colCount;
  
  for(page = 0; page < LCD_NUM_PAGES; page++)
  {
      ece315_lcdSetPage(page) ;

      for( colCount=0; colCount< LCD_NUM_COL; colCount++)
      {
        ece315_lcdSetColumn(colCount);
        ece315_lcdWriteData(0x00);
      }
    }
 }

//*****************************************************************************
// Each character is 10 columns wide.  The colStart is the column number where
// the first column will be printed.
//
// font.c contains a lookup table for printing out characters to the LCD screen.
// You should note that each character is 16 pixels high and 10 pixels wide.
// For each character, you will have to write to two different pages to print
// out a single character.  
//*****************************************************************************
void ece315_lcdWriteChar( uint8_t page, char c, uint8_t colStart)
 {
   uint16_t i, index;
    uint8_t upperPage, lowerPage;
    
    switch (page)
    {
      case 0:
      {
        upperPage = 0;
        lowerPage = 1;
        break;
      }
      
       case 1:
      {
        upperPage = 2;
        lowerPage = 3;
        break;
      }
      
       case 2:
      {
        upperPage = 4;
        lowerPage = 5;
        break;
      }
      
      case 3:
      {
        upperPage = 6;
        lowerPage = 7;
        break;
      }
    }
    if(c >= 32 || c <= 126)
    {
      
      index = c - 32;
      index = index * 20;
      
      ece315_lcdSetPage(upperPage) ;
      
      for( i=0; i< 10; i++)
      {
        ece315_lcdSetColumn(colStart*10 + i);
        ece315_lcdWriteData(courierNew_10ptBitmaps[index+i]);
      }
      
      ece315_lcdSetPage(lowerPage) ;
      
      for( i=10; i< 20; i++)
      {
        ece315_lcdSetColumn(colStart*10 + (i-10));
        ece315_lcdWriteData(courierNew_10ptBitmaps[index+i]);
      }

    }
    else
    {
      return;
    }
 }
 
//*****************************************************************************
// Write a string of characters out to the LCD screen.  Only the first 
// 10 characters will be printed.  The function will also terminate when
// a null character is encountered.
//*****************************************************************************
void ece315_lcdWriteString( uint8_t line, char *string)
{
  int i;
	int column = 0;
	for(i = 0; i < 10; i++){
		if(string[i] == '\n'){
			break;
		}
		else {
			ece315_lcdWriteChar(line, string[i], column);
			column++;
		}
	}
}  

