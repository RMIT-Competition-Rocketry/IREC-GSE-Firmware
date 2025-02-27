# IREC-GSE-Firmware

The GSE firmware architecture revolves around pre-existing libraries found in the Australis-Avionics-Firmware Repo, from which is responsible for the control of Hybrid Rocket Ground Support Equipment (GSE), all safety critical mechanisms, status indication, telemetry and data logging of state inputs. 

Due to the safety critical nature of the GSE and its responsibility to safe-guarding volatile gases such as O2 and N2O under high pressures near explosives, the code responsible for controlling these mechanisms are written in C in simple control flow logic. This therefore avoids the use of FreeRTOS and opts for simplicity of operation, utilising direct register control over a hardware abstractive approach to ensure more precise control over hardware peripherals while preventing unnecessary firmware bloat. 

**Current** features include:
Interrupt LoRa RX packet management of different payload types, coupled with correspondant TX communication to other ground support equipment utilised for launches
RX timer cutoff controlled interrupts that auto default to a purge state when remote RX commands are not received within 500ms-1s time interval -> this means deliberate effort is required to initiate any non-purge state
Dynamic diagnostic processing and error handling in case of error state

FOR BINARIES: Go into the debug folder -> then look for GSE_2.bin. The makefile is also present here. 


**TODO**:
Include SPI based data logging via non-volatile NAND storage housing state flags, error flags, key global variables, and sensor information.
Include USART data extraction from the flash storage via USART->USB
More sophisticated error management 
Make code base more modular to allow for future hardware revisions.

