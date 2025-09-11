# flash firmware
avrdude -c usbasp -p m328p  -U flash:w:/path/to/firmware.hex:i
avrdude -c usbasp -p m328p  -U flash:w:/home/arul/Documents/PlatformIO/Projects/Falcon_alert/.pio/build/ATmega328P/firmware.elf:i

#read fuse settings in hex
avrdude -c usbasp -p m328p -U lfuse:r:-:h -U hfuse:r:-:h -U efuse:r:-:h
# write lfuse
avrdude -c usbasp -p m328p -U lfuse:w:0xFF:m  #(default for arduino)
avrdude -c usbasp -p m328p -U lfuse:w:0x62:m  #(changes for internal clock)


# latest avrdude exe path: (compiled local)
# /home/arul/Projects/opensource/avrdude/build_linux/src/avrdude


avrdude -c stk500 -p m328p  -U flash:w:/home/arul/Documents/PlatformIO/Projects/Falcon_alert/.pio/build/ATmega328P/firmware.elf:i
avrdude -c stk500 -p m328p  -U flash:w:/home/arul/Documents/PlatformIO/Projects/Falcon_alert/.pio/build/ATmega328P/firmware.elf:e -P /dev/ttyUSB1