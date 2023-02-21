AR=/usr/bin/arm-linux-gnueabihf-ar rc
RANLIB=/usr/bin/arm-linux-gnueabihf-gcc-ranlib
CPP=/usr/bin/arm-linux-gnueabihf-g++
STRIP=/usr/bin/arm-linux-gnueabihf-strip
CFLAGS = -I/usr/include/c++/5/ 
CFLAGS += -I/usr/include/c++/5/ 
CFLAGS += -Iinclude
CLIBS=-lwiringPi -lpthread -lm library/sockets.a 
EXEC=Truking-WT

DataGather:DataGather.o serial.o myLib.o CModbusTcp.o CD_network.o log.o NDIO.o
	$(CPP) -o $(EXEC)  $? $(CLIBS)
	$(STRIP) $(EXEC)

myLib.o: myLib.c myLib.h
	$(CPP) -c $? 

log.o: log.c log.h
	$(CPP) -c $?

NDIO.o: NDIO.c NDIO.h
	$(CPP) -c $?

serial.o: serial.c serial.h
	$(CPP) -c $? 

CModbusTcp.o: CModbusTcp.c CModbusTcp.h
	$(CPP) -c $? 

CD_network.o: CD_network.c CD_network.h
	$(CPP) -c $? 

DataGather.o: DataGather.c DataGather.h
	$(CPP) -c $? $(CFLAGS)

clean:
	rm -f *.o *.gch $(EXEC)
