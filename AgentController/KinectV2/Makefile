#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

all: kinectController.so

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm -f ./*.so *~
