# General description
Autonomous flood protection system, which can work for months from a single 18650 Li-ion battery.
Includes a battery charger as well as a HW monitoring circuit. Includes power MOSFET for external valve control circuit. Even if SW is stuck or microcontroller is dead you will hear beeping informing you that something is wrong.
Two protected guard inputs make a redundant and reliable system.

# Hardware
Both schematic and PCB were created with KiCAD. KiCAD project is located in the PCB subfolder. Project uses components from my [KiCAD_libs](https://github.com/Sl-Alex/KiCAD_libs) repository.<br />

# Software
Project is created in [EmBitz](https://www.embitz.org/). Project skeleton is created in STM32CubeMX.<br />

# Licensing
The whole project is available under GPL v3 license (see [LICENSE](LICENSE) file), except of files, which clearly mention other license type in the file header.