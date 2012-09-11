avrdude -p m2560 -P COM146 -b 57600 -c stk500v2 -D -Uflash:w:Elisa3-avr-studio.hex:i -v
pause