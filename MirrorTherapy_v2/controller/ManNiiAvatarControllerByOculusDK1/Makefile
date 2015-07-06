# target file (.so)
TARGET = ./ManNiiAvatarControllerByOculus.so

# sources
SRCS =\
	$(COMMON)SigCmn.cpp \
	$(COMMON)Quaternion.cpp \
	$(COMMON_DEVICE)SensorData.cpp \
	$(COMMON_DEVICE)OculusRiftDK1SensorData.cpp \
	$(CONTROLLER_COMMON)Posture.cpp \
	$(CONTROLLER_COMMON)ManNiiPosture.cpp \
	$(CONTROLLER_COMMON)AvatarController.cpp \
	$(CONTROLLER_COMMON)ManNiiAvatarController.cpp \
	$(CURRENT)ManNiiAvatarControllerByOculus.cpp \

# source directories
CURRENT = ./
COMMON = ../../common/
COMMON_DEVICE = ../../common/device/
CONTROLLER_COMMON = ../ControllerCommon/

# sigverse path
SIG_SRC = $(SIGVERSE_PATH)/include

# compiler
CXX = g++

# compile option
CFLAGS = -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -fPIC -shared

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
