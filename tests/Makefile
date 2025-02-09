# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -g -DDEBUG

# Directories
SRC_DIR = ../src
TEST_DIR = .
BUILD_DIR = build

TARGET ?= playground.cpp

# Source and test files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
TEST_FILES = $(TEST_DIR)/$(TARGET)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(TEST_FILES))

# Executable
TEST_EXE = $(BUILD_DIR)/$(basename $(TARGET)).exe

# Default target (normal build)
all: $(TEST_EXE)

# Build test executable
$(TEST_EXE): $(OBJ_FILES) $(TEST_OBJ_FILES)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@
	@echo "Build complete: $(TEST_EXE)"

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files
$(BUILD_DIR)/%.o: $(TEST_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build files"

# Run tests
run: $(TEST_EXE)
	./$(TEST_EXE)
