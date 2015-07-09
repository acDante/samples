# target file (.so)
TARGET = ./OpenHouse2015Controller.so

# sources
SRCS =\
	$(SIGVERSE_BUILD_PATH)/sigverse/common/SigCmn.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/Quaternion.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/device/SensorData.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/device/KinectV2SensorData.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/common/device/OculusRiftDK1SensorData.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/Posture.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/ManNiiPosture.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/AvatarController.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/ManNiiAvatarController.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/device/KinectV2DeviceManager.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/common/device/OculusDK1DeviceManager.cpp \
	$(SIGVERSE_BUILD_PATH)/sigverse/controller/OpenHouse2015Controller/OpenHouse2015Controller.cpp \

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
