#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

OBJS     = cleanUpRobot.so cleanUpRobot2.so cleanUpUser.so cleanUpUser2.so TrashBox.so Wagon.so Table.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm ./*.so
