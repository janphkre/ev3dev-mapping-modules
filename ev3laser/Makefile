TARGET = ev3laser
OBJS = main.o ev3dev.o net_udp.o misc.o xv11lidar.o
EV3DEV = ../lib/ev3dev-lang-cpp
SHARED = ../lib/shared
XV11LIDAR = ../lib/xv11lidar
INCLUDE = ../lib

CC = gcc
CXX = g++
DEBUG = 
CFLAGS = -O2 -Wall -DEV3 -c -I $(INCLUDE)
CXX_FLAGS = -O2 -std=c++11 -Wall -DEV3 -D_GLIBCXX_USE_NANOSLEEP -c $(DEBUG) -I $(INCLUDE)
LFLAGS = -Wall $(DEBUG)

$(TARGET) : $(OBJS)
	$(CXX) $(LFLAGS) $(OBJS) -o $(TARGET)

main.o : main.cpp $(EV3DEV)/ev3dev.h $(SHARED)/misc.h $(SHARED)/net_udp.h  $(XV11LIDAR)/xv11lidar.h 
	$(CXX) $(CXX_FLAGS) main.cpp

ev3dev.o : $(EV3DEV)/ev3dev.h $(EV3DEV)/ev3dev.cpp 
	$(CXX) $(CXX_FLAGS) $(EV3DEV)/ev3dev.cpp

misc.o : $(SHARED)/misc.h $(SHARED)/misc.cpp
	$(CXX) $(CXX_FLAGS) $(SHARED)/misc.cpp
	
net_udp.o: $(SHARED)/net_udp.h $(SHARED)/net_udp.cpp $(SHARED)/misc.h
	$(CXX) $(CXX_FLAGS) $(SHARED)/net_udp.cpp

xv11lidar.o: $(XV11LIDAR)/xv11lidar.h $(XV11LIDAR)/xv11lidar.c
	$(CC) $(CFLAGS) $(XV11LIDAR)/xv11lidar.c

clean:
	\rm -f *.o $(TARGET)