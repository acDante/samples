#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

OBJS     = user.so partner.so point_record.so user2.so partner2.so user3.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
