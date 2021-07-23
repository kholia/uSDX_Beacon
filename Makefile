# https://github.com/arduino/arduino-cli/releases

fqbn := arduino:avr:uno

default:
	arduino-cli compile --warnings all --fqbn=${fqbn} -e uSDX_Beacon
	avrdude -v -patmega328p -cusbasp -Pusb -Uflash:w:uSDX_Beacon/build/arduino.avr.uno/uSDX_Beacon.ino.hex:i

install_platform:
	arduino-cli core install arduino:avr

deps:
	# arduino-cli lib install "Etherkit JTEncode"  # not great!
	arduino-cli lib install "Etherkit Si5351"
	arduino-cli lib install "RTClib"
	arduino-cli lib install "Time"
	arduino-cli lib install "LiquidCrystal"

init:
	# http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p
	avrdude -v -patmega328p -cusbasp-clone -Pusb -e -Ulock:w:0xFF:m -Uefuse:w:0xFF:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m


install_arduino_cli:
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
