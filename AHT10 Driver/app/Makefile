arm = arm-linux-gnueabihf-gcc
objects = ahtApp
source = ahtApp.c
main: ahtApp.o
	$(arm) -o $(objects) ahtApp.o
ahtApp.o: $(source)
	$(arm) -c $(source)
clean:
	rm *.o
	rm $(objects)
