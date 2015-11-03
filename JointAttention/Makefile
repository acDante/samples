#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

OBJS     = user.so partner.so point_record.so user2.so partner2.so user3.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm ./*.so
