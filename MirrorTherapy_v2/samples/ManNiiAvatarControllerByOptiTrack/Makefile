# target file (.so)
TARGET = ./ManNiiAvatarControllerByOptiTrack.so

# sources
SRCS =\
	$(SIGVERSE_BUILD_PATH)/sigverse/common/SigCmn.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/Quaternion.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/device/SensorData.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/device/OptiTrackSensorData.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/Posture.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/ManNiiPosture.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/AvatarController.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/ManNiiAvatarController.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/device/OptiTrackDeviceManager.cpp \
	ManNiiAvatarControllerByOptiTrack.cpp \

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
