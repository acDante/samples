#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include


OBJS     =  distanceSensor.so
all: $(OBJS)


#compile
./%.so: ./%.cpp
#	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -fPIC -shared -o $@ $<
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
