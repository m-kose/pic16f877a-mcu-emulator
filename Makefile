# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++11 -Wall -g

# Source files and object files
SRCS = main.cpp register.cpp memory_banks.cpp cpu.cpp
OBJS = $(SRCS:.cpp=.o)

# Executable name
TARGET = pic_simulator

# Default rule
all: $(TARGET)

# Rule to link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Rule to compile source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

# Rule to run the executable
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
