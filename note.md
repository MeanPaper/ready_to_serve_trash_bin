# PCB updates note

[Update]: ESP32-S3 MCU control circuit version 2
- Made pin assignment for motor connnections 
- Each DC motor uses 5 pins (Motor Hall Phase A & B, Motor direction control port A & B, PWM pin)
- Linear Actuator uses 3 pins (PWM pin, direcetion control pin)
- Added testing output pins for the MCU control circuit (for motors specifically)
- Reduced the numbers of ports for other programmable GPIOs
- The output pins are Connector_Pinhead (male pins)

[Update]: Update filenames and add a quick note
- `esp32s3-chip-only-sch-v1.zip`: ESP32S3 minimal schematic configuration (the basic layout for the chip to work )
- `power-and-motors-pcb-v1.zip`: power circuit and motor driver pcb schematic and layout
- `ready-to-serve-trash-bin-v0.zip`: the basic version of the PCB
- `ready-to-serve-trash-bin-v1.zip`: the current version of the PCB and schematic, physical PCB and components are ready for testing 
- `ready-to-serve-trash-bin-v2.zip`: the new PCB layout, same schematics but different arrangement of components on the PCB (by Josh)
- `ready-to-serve-trash-bin-v3.zip`: the final version of the PCB (by Josh)
