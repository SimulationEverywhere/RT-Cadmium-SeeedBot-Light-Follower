CC=g++
CFLAGS=-std=c++17
COMPILE_TARGET=NUCLEO_F401RE
FLASH_TARGET=NODE_F401RE
EXECUTABLE_NAME=SEEED_BOT_TOP

INCLUDECADMIUM=-I ../../cadmium/include
INCLUDEDESTIMES=-I ../../cadmium/DESTimes/include

embedded:
	mbed compile --target $(COMPILE_TARGET) --toolchain GCC_ARM --profile ../cadmium.json

flash:
	sudo cp ../BUILD/$(COMPILE_TARGET)/GCC_ARM-CADMIUM/*.bin /media/$(USER)/$(FLASH_TARGET)/
	$(info *** FLASH MAKE TAKE ~10 Seconds! DO NOT RESET WHILE COM PORT LED IS FLASHING! ***)

all: main.o 
	$(CC) -g -o $(EXECUTABLE_NAME) main.o 

main.o: main.cpp
	$(CC) -g -c $(CFLAGS) $(INCLUDECADMIUM) $(INCLUDEDESTIMES) main.cpp -o main.o

clean:
	rm -f $(EXECUTABLE_NAME) *.o *~

eclean:
	rm -rf ../BUILD
