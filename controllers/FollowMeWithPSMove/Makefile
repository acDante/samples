#sigverse header
INCLUDE_PATH = $(SIGVERSE_PATH)/include

OBJS     = Camera.so  CheckPoint2.so  CheckPoint.so  FinishLine.so  FollowMeHuman.so  FollowMeTestRobot.so  ITP1.so  ITP2.so  Moderator.so  Operator.so  score.so  Wall.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -Wall -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(INCLUDE_PATH) -fPIC -shared -o $@ $<

clean:
	rm -f ./*.so
