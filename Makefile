# COMP/ELEC/MECH 450/550 - Project 3  (2025) Makefile
# RTP Implementation and 2D Planning

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
INCLUDES = -I/usr/local/include/ompl-1.7 -I/usr/include/eigen3 -Isrc
LIBS = -L/usr/local/lib -lompl -lboost_system -lboost_filesystem

# Source files for 
EXERCISE2_SOURCES = src/Project3Exercise2.cpp src/RTP.cpp src/CollisionChecking.cpp

# Object files
EXERCISE2_OBJECTS = $(EXERCISE2_SOURCES:.cpp=.o)

# Targets for 
TARGETS = Project3Exercise2

.PHONY: all setup clean verify help

all: setup $(TARGETS)

setup:
	@echo "Setup complete - no additional dependencies needed for "

# Exercise 2: 2D Planning with RTP Implementation
Project3Exercise2: $(EXERCISE2_OBJECTS)
	@echo "Building Project3Exercise2..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)
	@echo "Build complete: Project3Exercise2"

# Compile source files
%.o: %.cpp
	@echo "Compiling $<..."
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $@

# Verify OMPL installation
verify:
	@echo "Verifying OMPL installation..."
	@echo '#include <ompl/config.h>' > test_ompl.cpp
	@echo '#include <iostream>' >> test_ompl.cpp
	@echo 'int main() { std::cout << "OMPL version: " << OMPL_VERSION << std::endl; return 0; }' >> test_ompl.cpp
	@$(CXX) $(CXXFLAGS) $(INCLUDES) test_ompl.cpp -o test_ompl $(LIBS) && ./test_ompl && rm -f test_ompl test_ompl.cpp
	@echo "OMPL verification complete!"

# Clean build files
clean:
	@echo "Cleaning..."
	rm -f src/*.o $(TARGETS) *_path.txt

help:
	@echo "Available commands:"
	@echo "  make                     - Install dependencies and build Exercise 2"
	@echo "  make setup               - Setup (no additional dependencies for )"
	@echo "  make Project3Exercise2   - Build Exercise 2 (2D Planning with RTP)"
	@echo "  make verify              - Check OMPL installation"
	@echo "  make clean               - Remove build files"
	@echo "  make help                - Show this help"
	@echo ""
	@echo "Project 3  - Quick Start:"
	@echo "  1. Implement RTP algorithm in src/RTP.h and src/RTP.cpp"
	@echo "  2. Complete Exercise 2 in src/Project3Exercise2.cpp"
	@echo "  3. Build and run: make && ./Project3Exercise2"
