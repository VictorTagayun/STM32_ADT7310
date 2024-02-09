# Interfacing [ADT7310](https://www.analog.com/media/en/technical-documentation/data-sheets/adt7310.pdf) to STM32 via SPI

[ADT7310](https://www.analog.com/media/en/technical-documentation/data-sheets/adt7310.pdf) is Analog Devices' Digital SPI Temperature Sensor with ±0.5°C accuracy and 16-bit ADC. Here we interface and program the ADT7310 but encountered hiccups along the way. In the end it was solved and hopefully it can help others too. So follow me along the way in interfacing to ADT7310.

## Datasheet

![Datasheet](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/datasheet00.png)

## Project files  

* NUCLEO-G474RE_ADT7310_V1  = Initial troubleshooting and building up code and basic reading of registers
* NUCLEO-G474RE_ADT7310_V2  = Writing to registers, reading 13-bit and 16-bit Temperature. Streamlined code and avoid use of global variablea inside functions.

### My first encounter, reading ***Status*** register

I expect a **0x80** after power up. But it is always giving **0x00** as shown below. This is after I send a **0x40** (read **Status**).

![0x00](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/20240209_000229.jpg)

* CH1 (yellow)   = CS
* Ch2 (magenta)  = CLK
* CH3 (pink)     = MOSI
* CH4 (blue)     = MISO

As per specs, the **CLK** should rest high, I suppose it should also start high. But from the above figure, CLK "somewhat" started low.

![CLK](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/datasheet01.png)

I changed the code and add **pull-up** on the **CLK** pin

from
```
GPIO_InitStruct.Pull = GPIO_NOPULL;
```
to 
```
GPIO_InitStruct.Pull = GPIO_PULLUP;
```

Now the waveform becomes like as shown below. The **CLK** now starts high.

![Pullup](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/20240209_001324.jpg)


This code change did not solve the problem as I printed through serial for each register's data. All register's data are all **0s**.

![](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/Serial01.png)

After re-reading the datasheet, I may need to reset the ADT7130 via serial (AKA software).

![serialReset](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/datasheet02.png)

After sending 32bit of **highs** or **1s** and even if the **CLK** starts at **low** as shown below, the default value of all registers matches the specs.

![32highs](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/20240209_001953.jpg)

Here now are the default values read after serial/software reset.

![default](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/Serial02.png)


### V2, write to registers and read 13bit and 16bit temperature data

Here is the serial data when writing to the registers and changing from 13bit and 16bit temperature data.

![V2serial](https://github.com/VictorTagayun/STM32_ADT7310/blob/main/photos/Serial03.png)

## ***........... still a work in progress  ...........***


### Other related topics : 

[Interface to Texas Instruments' INA239 and INA229](https://github.com/VictorTagayun/STM32_INA239-INA229)

[SPI Master Receive and Slave Transmit DMA using 2 SPI Channels on a Single Board](https://github.com/VictorTagayun/STM32_SPI-Master-RX-DMA_SingleBoard)

[STM32 SPI DMA Retransmit](https://github.com/VictorTagayun/STM32_SPI_DMA_Retransmit)

[SPI Speed of DMA vs Interrupt](https://github.com/VictorTagayun/SPI_DMA_VS_Interrupt)

*Disclaimer:*
[Updated Disclaimer](https://github.com/VictorTagayun/GlobalDisclaimer)

*The projects posted here are for my Personal reference, learning and educational purposes only.*
*The purpose of a certain project may be for testing a module and may be just a part of a whole project.*
*It should not be used in a production or commercial environment.*
*Any cause of injury and/or death is the sole responsibility of the user.*

### Go to my github.io : [https://victortagayun.github.io/](https://victortagayun.github.io/)
