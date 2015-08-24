# target file (.so)
TARGET = ./RHIBallController.so

# sources
SRCS =\
	$(SIGVERSE_BUILD_PATH)/sigverse/common/SigCmn.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/RHIBallController/RHIBallController.cpp \

# sigverse path
SIG_SRC = $(SIGVERSE_PATH)/include

# compiler
CXX = g++

# compile option
CFLAGS = -std=c++11 -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIGVERSE_BUILD_PATH) -fPIC -shared

# generate intermediate object file from sources.
OBJS := $(SRCS:.cpp=.o)

# -- compile rule --

all: $(TARGET)

# intermediate output file
%.o: %.cpp
	$(CXX) $(CFLAGS) -o $@ -c $<

# target file(.so)
$(TARGET): $(OBJS) $(SRCS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) # Link object files.
	rm -f $(OBJS) # Remove intermediate files.

# remove object file
clean:
	rm -f $(TARGET) $(OBJS)
