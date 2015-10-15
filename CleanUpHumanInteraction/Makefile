#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

#オブジェクトファイルの指定
OBJS     = cleanUpRobot.so cleanUpRobot2.so cleanUpUser.so cleanUpUser2.so TrashBox.so Wagon.so Table.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
