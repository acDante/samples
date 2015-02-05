#SIGVerse header file
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

#Target
OBJS     = OculusRiftDK2Ctrl.so

all: $(OBJS)

#Compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller  -fPIC -shared -o $@   $<

clean:
	rm $(OBJS)
