SOURCE_C_FILE += $(shell find . -name "*.c")
SOURCE_CPP_FILE += $(shell find . -name "*.cpp")

TARGET_PATH += .
COMPILE_OPTION := -fPIC -shared -fvisibility=hidden -flto -fwrapv -Wno-deprecated -Wno-parentheses -Wno-unused-result -Wreturn-type -fno-strict-aliasing
MACRO := -D_REENTRANT -DDECLSPEC_DLL_EXPORT -DCCT_NUM_DOUBLE

DEFAULT_LINK := -pthread -lm
ifeq ($(shell uname), Linux)
endif

COMPILER := gcc
ifeq ($(COMPILER), gcc)
SOURCE_CPP_FILE :=
endif

DEBUG_TARGET := $(TARGET_PATH)/libCollisionDetectionDebug.so
ASAN_TARGET := $(TARGET_PATH)/libCollisionDetectionAsan.so
RELEASE_TARGET := $(TARGET_PATH)/libCollisionDetection.so

all: debug asan release

debug:
	$(COMPILER) $(MACRO) -D_DEBUG -g $(COMPILE_OPTION) $(SOURCE_C_FILE) $(SOURCE_CPP_FILE) -o $(DEBUG_TARGET) $(DEFAULT_LINK)

asan:
	$(COMPILER) $(MACRO) -D_DEBUG -g -fsanitize=address $(COMPILE_OPTION) $(SOURCE_C_FILE) $(SOURCE_CPP_FILE) -o $(ASAN_TARGET) $(DEFAULT_LINK)

release:
	$(COMPILER) $(MACRO) -DNDEBUG -O2 $(COMPILE_OPTION) $(SOURCE_C_FILE) $(SOURCE_CPP_FILE) -o $(RELEASE_TARGET) $(DEFAULT_LINK)
