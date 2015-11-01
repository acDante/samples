#sigverse header
INCLUDE_PATH  = $(SIGVERSE_PATH)/include

OBJS     = AgentController.so AgentController2.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm ./*.so *~
