# Host-side tests for Zephyr-free firmware code (no Zephyr SDK needed).
CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -O1 -g

all: run

awProtocolTest: awProtocolTest.cpp ../../src/sections/awProtocol.hpp
	$(CXX) $(CXXFLAGS) -o $@ awProtocolTest.cpp

rfuProtocolTest: rfuProtocolTest.cpp ../../src/sections/rfuProtocol.hpp
	$(CXX) $(CXXFLAGS) -o $@ rfuProtocolTest.cpp

# Independent derivation of the SIO32 ID-exchange sequence (no firmware deps);
# proves the canned table converges FRLG's checkID to RFU_ID.
rfuIdDerive: rfuIdDerive.cpp
	$(CXX) $(CXXFLAGS) -o $@ rfuIdDerive.cpp

run: awProtocolTest rfuProtocolTest rfuIdDerive
	./awProtocolTest
	./rfuProtocolTest
	./rfuIdDerive

clean:
	rm -f awProtocolTest rfuProtocolTest rfuIdDerive

.PHONY: all run clean
