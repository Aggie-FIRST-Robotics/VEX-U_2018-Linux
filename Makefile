CC = g++
CFLAGS = -std=c++11 -g
INCLUDE_DIRS = -I/usr/include/spinnaker/ -I$(CURDIR)/include/
LDFLAGS = -lSpinnaker -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lserial -pthread
TARGET = VEXU_2019_Targeting

SRC_DIR = src
SRC = $(wildcard $(SRC_DIR)/**/*.cpp) $(wildcard $(SRC_DIR)/*.cpp) 
BUILD_DIR = build
OBJ = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(SRC))
OBJ_DIRS = $(dir $(OBJ))

$(TARGET): $(OBJ)
	$(CC) $^ -o $@  $(LDFLAGS)

$(BUILD_DIR)/%.o: %.cpp
	$(CC) -c $< $(INCLUDE_DIRS) $(CFLAGS) -o $@

all: $(OBJ)

$(OBJ): | $(OBJ_DIRS)

$(OBJ_DIRS):
	mkdir -p $(OBJ_DIRS)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET)

.PHONY: test
test:
	$(info $(SRC))
	$(info $(OBJ))
	$(info $(OBJ_DIRS))
