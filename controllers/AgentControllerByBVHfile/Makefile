#SIGVerseソースの場所指定
SIG_SRC  = $(SIGVERSE_PATH)/include

#オブジェクトファイルの指定
OBJS     =  BVH.o BVHController.so BVHController.o

all: $(OBJS)

#コンパイルを行います。
BVHController.o: ./srcs/BVHController.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -c -o $@   $<

BVH.o: ./srcs/BVH.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -c -o $@ $<

BVHController.so: BVHController.o BVH.o
	g++ -shared -o BVHController.so BVHController.o BVH.o

clean:
	rm BVHController.so BVHController.o BVH.o
