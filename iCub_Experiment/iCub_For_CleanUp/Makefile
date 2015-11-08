#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

OBJS     = cleanUpRobot2.so cleanUpUser.so cleanUpUser2.so moderator.so Camera.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm -f ./*.so
	rm -f ./*~
