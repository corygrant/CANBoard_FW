# CANBoard_FW
CANBoard is a simple CAN enabled IO board, specifically designed to be used in devices like steering wheels/button boxes/panels/etc.

* 8 digital inputs
    * Ground switching
* 5 analog inputs
    * 5V max
* 4 digital outputs
    * Low side switch (open collector)
    * 0.5A max each
* CAN output message format compatible with ECUMaster CAN Switch Board
* STM32F303K8

# Versions
Current software versions used:
* CubeIDE V1.11.2
* CubeMX V6.7.0
* FW_F3 V1.11.3

# Goals
- Create a low cost device that my friends and I can use in our project cars
- Use components that are easily soldered by hand (hence 0805) and are preferably from my stock of frequently used parts (ex: LD1117S33CTR)
- Format the CAN messages in a similar way to the ECUMaster CAN Switch board
- Share my work with others for reference, inspiration or collaboration. 

If this project does help you in any way, I'd appreciate a message!

# Disclaimer
This is a personal hobby project. I am not a professional. Use at your own risk. 

# CAN
Currently configured as 500 Kbps only

# Message Format
The BASE_ID is set using the 1 and 2 solder jumpers, see [CANBoard hardware](https://github.com/corygrant/CANBoard_HW) for more info

![Message Format](/Images/CANBoard_V2_MessageFormat.jpg)

* **Analog Inputs** : millivolt readings
* **STM32 Temperature** : Temperature in celsius as reported by STM32
* **Rotary** : Position of a rotary switch wired to each analog input 
    * *Needs work on scaling for different numbers of positions*
* **Digital Inputs** : Bits representing the state of each switch input
* **Analog/Dig Inputs** : Bits representing the analog inputs used as digital inputs
    * Value over half scale (2.5V) the input is true (1)
* **Output States** : Bits representing the digital output states
* **Heartbeat** : Value incremented every transmit (uint8_t)

# Hardware
[CANBoard hardware](https://github.com/corygrant/CANBoard_HW)