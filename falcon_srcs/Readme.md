IDE setup:
1. Install Visual Studio Code (https://code.visualstudio.com/download)
2. Install PlatformIO IDE extension (follow https://platformio.org/install/ide?install=vscode)
3. Extract the project zip file attached with the mail
4. Open the extracted project folder in Vscode and open platformio.ini file, IDE will setup the project from the platformio.ini config file
    or
   Please use Open project option from PIO home.

Programming software setup: (AVRDUDE)
1. install avrdude (please follow tutorial https://youtu.be/QiIVD196aD8?t=97)

Updating the serial port:
1. Please update the upload_port value in platformio.ini file (it may change on every connection). So please make sure it get updated before flashing the board.
2. To find the serial port number
    - Connect the Programmer USB to laptop 
    - open Device Manager  (can use search bar )
    - expand the Ports (COM & LTP) menu item
    - Find the new device listed 
    - e.g. port number (COM4)

Build project code:
1. Use build menu from bottom status bar or use (PlatformIO -> Project Tasks -> ATmega328P -> General -> Build) to build the code

Flashing the firmware:
1. Attach the TAG connector with PCB 
2. Power on the device
3. Use Upload menu from bottom status bar or use (PlatformIO -> Project Tasks -> ATmega328P -> General -> Upload) to flash the code
4. Please wait till the screen print done/success 
5. once done, connector cable can be removed and its ready for colleting logs

Alternate cli option to upload the firmware:
avrdude.exe -c stk500 -p m328p -P COM4  -U flash:w:{PATH TO firmware.elf FILE}:e 

e.g.: avrdude -c stk500 -p m328p -P COM4 -U flash:w:firmware.elf:e 


=======================================================================

Collecting Logs:
1. Connect the serial cable to PCB 
    - Pin connection (cable tx pin -> pcb rx pin, cable rx pin -> pcb tx pin, cable gnd -> pcb gnd)
2. connect the USB side of cable to laptop
3. once the serial usb connected find the serial port number
    - Connect the Programmer USB to laptop 
    - open Device Manager  (can use search bar )
    - expand the Ports (COM & LTP) menu item
    - Find the new device listed 
    - e.g. port number (COM4)
    ------
    - if the serial port not visible in  (COM & LTP) menu please search for CP2102 usb-to-serial bridge driver and install it
4. open putty and select serial connection type and fill detail
    - Serial line: serial port number (eg. COM4)
    - Speed : 115200
5. configure logging (please select log all the session output option under Session -> Logging -> All session output)
6. click Open button in the botton of window
7. Happy logging



# avrdude -c stk500 -pm328p -P /dev/ttyUSB0 -U lfuse:r:-:i -v
# avrdude -c stk500 -pm328p -P /dev/ttyUSB0 -U lfuse:w:0xFF:m -v
# minicom -D /dev/ttyUSB1 -O timestamp=extended -C test3.log