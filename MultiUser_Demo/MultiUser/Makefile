#sigverse header
INCLUDE_PATH  = $(SIGVERSE_PATH)/include

OBJS     = cleanUpUser.so cleanUpUser2.so MultiUserCtr.so Camera.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm ./*.so
