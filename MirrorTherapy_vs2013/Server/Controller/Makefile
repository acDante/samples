# 生成する.soファイル名
TARGET = ./ManNiiAvatarController.so

# コンパイルしたいソースコードのファイル名
# ・ソースコードを変更したりする場合はここの記述を変更する
# ・「\」をファイル名の末尾に加えて記述する
SRCS =\
	$(DIR)AvatarPosture.cpp \
	$(DIR)AvatarController.cpp \
	$(DIR)ManNiiAvatarPosture.cpp \
	$(DIR)ManNiiAvatarController.cpp \

# ソースコードのある場所
DIR = ./

# sigverseのパス
SIG_SRC  = $(SIGVERSE_PATH)/include

# コンパイラ
CXX = g++

# コンパイルオプションなど
CFLAGS = -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -fPIC -shared 

# 中間ファイル名をソースコードのファイル名から生成する（.cpp -> .o）
OBJS := $(SRCS:.cpp=.o)

# ここまでは変数の定義
# ここからはコンパイルのルールを記述

# make に対する引数が無いときのルール
all: $(TARGET)

# 中間ファイルを生成するルール
.cpp.o:
	@$(CXX) $(CFLAGS) -c $< # Generate object file

# .soファイルを生成するルール
$(TARGET): $(OBJS) $(SRCS)
	@$(CXX) $(CFLAGS) -o $@ $(OBJS) # Link object files.
	@rm -f $(OBJS) # Remove intermediate files.

# 生成したファイルを削除するルール
clean:
	rm -f $(TARGET) $(OBJS)
