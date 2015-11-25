#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

OBJS = ControllerSample.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm ./*.so
