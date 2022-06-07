# CFAF240400C1-030Tx

This is example Seeeduino (Arduino clone) software for the Crystalfontz CFAF240400C1-030Tx family of displays.

This display is a 3 inch, 240x400, full color, TFT. It come as a non-touch or can have a capacitive touch screen attached to the display. The display is driven by a [Sitronix ST7796s display controller](https://www.crystalfontz.com/controllers/Sitronix/ST7796S/) and the capacitive touch is controlled by a [FocalTech FT636G](https://www.crystalfontz.com/controllers/FocalTech/FT6336G/).

   
  
These products can be found here:\
Cap Touch - [CFAF240400C1-030TC](https://www.crystalfontz.com/product/cfaf240400c1030tc)\
Non Touch - [CFAF240400C1-030TN](https://www.crystalfontz.com/product/cfaf240400c1030tn)

Additionally, these displays are available with an EVE controller:\
Cap Touch with EVE FT811 - [CFA240400E0-030TC](https://www.crystalfontz.com/product/cfa240400e0030tc)\
Non Touch with EVE FT811 - [CFA240400E0-030TN](https://www.crystalfontz.com/product/cfaf240400c0030tna11)


# Connection Details

### LCD control lines
|  ARD      | Port  | CFAF240400C1    |  Function - Parallel
|-----------|-------|-----------------|-----------------------------
| 3.3V      |       | #1-2,33-34,42   |  POWER 3.3V
| GND	      |       | #3,5-8,28,35-41 |  GROUND
| A0        | PORTC | #29             |  Read                  (RD)
| A1        | PORTC | #30             |  Write                 (WR)
| A2        | PORTC | #31             |  Register Select       (RS)
| A3        | PORTC | #4              |  Reset              (RESET)
| D8        | PORTC | #32             |  Chip Enable Signal    (CS)
|           | PORTC | #29             |  Back Light            (BL)
| D0        | PORTD | #26             |  LCD_D10              (DB0)
| D1        | PORTD | #25             |  LCD_D11              (DB1)
| D2        | PORTD | #24             |  LCD_D12              (DB2)
| D3        | PORTD | #23             |  LCD_D13              (DB3)
| D4        | PORTD | #22             |  LCD_D14              (DB4)
| D5        | PORTD | #21             |  LCD_D15              (DB5)
| D6        | PORTD | #20             |  LCD_D16              (DB6)
| D7        | PORTD | #19             |  LCD_D17              (DB7)


### Cap Touch control lines
|  ARD      | Port  | CFAF240400C1    |  Function - Parallel
|-----------|-------|-----------------|-----------------------------
| 3.3V      |       | #2,3            |  POWER 3.3V
| GND	      |       | #1,8            |  GROUND
| D9        | PORTC | #6              |  Interrupt      (INTERRUPT)
| 3.3V      | PORTC | #7              |  Reset          (TCH_RESET)
| SDA       | PORTC | #5              |  I2C SDA          (I2C_SDA)
| SCL       | PORTD | #4              |  I2C SCL          (I2C_SCL)

### Micro SD control lines
| microSD Pin | Seeeduino Pin| Connection Description |
|-------------|--------------|------------------------|
| 2 (CS)      | 10           | SD CS                  |
| 3 (DI)      | 11           | SD MOSI                |
| 4 (VDD)     | 3.3V         | +3.3V Power            |
| 5 (SCLK)    | 13           | SD SCLK                |
| 6 (VSS)     | GND          | Ground                 |
| 7 (DO)      | 12           | SD MISO                |

[microSD](https://www.crystalfontz.com/product/cfa10112) connection is optional