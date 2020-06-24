# SHPI.zero Basic ATmega32u4 Firmware

Basic firmware for ATmega32u4 as I2C-Slave

- communication only over I2C (USB free for CULFW)
- backlight control
- LCD setup
- control relays
- control internal blower fan
- control LED
- read sensors and more
- uses counter0 for VENT PWM

## prerequisites
A SHPI.zero with GCC and DFU-programmer installed.



## Getting Started

Download GIT folder on your SHPI.zero:
```bash
git clone https://github.com/shpi/zero_avr_firmware_std.git
```



## I2C example command

Read relay 1 status

```bash
i2cget -y 2 0x2A 0x0D             

```
it should return :
```
0x00   // or 0xFF, depending relay 1 status 
```

If response consists more bytes, use

```bash
i2cget -y 2 0x2A    

```
to read one more byte.

Set RGB LED to white


```bash
i2cset -y 2 0x2A 0x8c 0xFF 0xFF 0xFF i    

```




## Compile & flash
To compile and flash:
```bash
cd zero_avr_firmware
sudo make flash
```

## I2C read commands									
										
|	value	|	commandbyte	|	in hex	|	response	|	format		|
|---------------|	----------	|--------------	|	----------	|	----------	|
|A0 (GD0 CC1101)|	0b00000000	|	0x00	|	10bit in 2 Byte	|	0 -1023		|
|A1 (MICS CO)	|	0b00000001	|	0x01	|	10bit in 2 Byte	|	0 -1023		|
|A2 (MICS No2)	|	0b00000010	|	0x02	|	10bit in 2 Byte	|	0 -1023		|
|A3 (MICS NH3	|	0b00000011	|	0x03	|	10bit in 2 Byte	|	0 -1023		|
|A4 (MP135)	|	0b00000100	|	0x04	|	10bit in 2 Byte	|	0 -1023		|
|A5 (NTC 10K)	|	0b00000101	|	0x05	|	10bit in 2 Byte	|	0 -1023		|
|A7 (ACS712)	|	0b00000110	|	0x06	|	10bit in 2 Byte	|	0 -1023		|
|BL_LEVEL	|	0b00000111	|	0x07	|	1 Byte		|	0-31		|
|VENT_RPM	|	0b00001000	|	0x08	|	2 Byte		|	UPM		|
|VCC Atmega	|	0b00001001	|	0x09	|	2 Byte		|	in Millivolts	|
|IntTemp Atmega	|	0b00001010	|	0x0A	|	2 Byte		|	degree celsius	|
|FREERAM	|	0b00001011	|	0x0B	|	2 Byte		|	in Bytes	|
|LED_COLOR	|	0b00001100	|	0x0C	|	3 Byte		|	R, G, B 0-255	|
|Relay1		|	0b00001101	|	0x0D	|	1 Byte		|	0x00 / 0xFF	|
|Relay2		|	0b00001110	|	0x0E	|	1 Byte		|	0x00 / 0xFF	|
|Relay3		|	0b00001111	|	0x0F	|	1 Byte		|	0x00 / 0xFF	|
|D13 (reset RPI)|	0b00010000	|	0x10	|	1 Byte		|	0x00 / 0xFF	|
|HWB (gasheater)|	0b00010001	|	0x11	|	1 Byte		|	0x00 / 0xFF	|
|Buzzer		|	0b00010010	|	0x12	|	1 Byte		|	0x00 / 0xFF	|
|VENT_PWM	|	0b00010011	|	0x13	|	1 Byte		|	0 – 255		|
|A7_VG(AC curr) |       0b00010110      |       0x14    |       1 Byte          |       0 – 1023         |

										
## I2C write commands									

|	command		|	commandbyte	|	in hex	|	expects	|	value				|
|	----------	|	----------	|----------	|----------	|	----------			|
|	BL_LEVEL_s	|	0b10000111	|	0x87	|	1 Byte	|	0 = off .. 31= 100%		|
|	LED_COLOR_s	|	0b10001100	|	0x8C	|	3 Byte	|	R, G, B 0-255			|
|	Relay1_S	|	0b10001101	|	0x8D	|	1 Byte	|	0x00 / 0xFF			|
|	Relay2_S	|	0b10001110	|	0x8E	|	1 Byte	|	0x00 / 0xFF			|
|	Relay3_S	|	0b10001111	|	0x8F	|	1 Byte	|	0x00 / 0xFF			|
|	D13_S		|	0b10010000	|	0x90	|	1 Byte	|	0x00 / 0xFF			|
|	HWB_S		|	0b10010001	|	0x91	|	1 Byte	|	0x00 / 0xFF			|
|	Buzzer_S	|	0b10010010	|	0x92	|	1 Byte	|	0x00 / 0xFF - NEW:0x01 for click|
|	VENT_PWM_S	|	0b10010011	|	0x93	|	1 Byte	|	0 = on .. 255 = off		|


## License

This project is licensed under the GPL v3 license.
