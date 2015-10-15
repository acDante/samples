#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include/sigverse

OBJS     = Camera.so  CheckPoint2.so  CheckPoint.so  FinishLine.so  FollowMeHuman.so  FollowMeTestRobot.so  ITP1.so  ITP2.so  Moderator.so  Operator.so  score.so  Wall.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so
