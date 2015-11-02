#SIGVerse header file
INCLUDE_PATH = $(SIGVERSE_PATH)/include

#Target
OBJS     = OculusRiftCtrl.so

all: $(OBJS)

#Compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DdDOUBLE -std=c++11 -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm $(OBJS)
