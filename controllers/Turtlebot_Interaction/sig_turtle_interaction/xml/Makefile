#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include

#オブジェクトファイルの指定
OBJS     = Human_avatar.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so