# Host-side tests for Zephyr-free firmware code (no Zephyr SDK needed).
CXX ?= g++
CXXFLAGS ?= -std=c++20 -Wall -Wextra -O1 -g

all: run

SRC := ../../src
HANDSHAKE_SRCS := handshakeTest.cpp $(SRC)/layers/packet/packetLayer.cpp $(SRC)/sections/usbSection.cpp \
                  $(wildcard $(SRC)/layers/packet/packetCommands/*.cpp)

awProtocolTest: awProtocolTest.cpp ../../src/sections/awProtocol.hpp
	$(CXX) $(CXXFLAGS) -o $@ awProtocolTest.cpp

# The real packet layer and USB section, over host stand-ins for Zephyr (stubs/)
# and a modelled GBA in place of the PIO link layer.
handshakeTest: $(HANDSHAKE_SRCS) $(wildcard stubs/*/*.h stubs/*/*/*.h) $(wildcard $(SRC)/layers/packet/*.hpp $(SRC)/sections/usbSection.hpp)
	$(CXX) $(CXXFLAGS) -Istubs -I$(SRC) -o $@ $(HANDSHAKE_SRCS) -lpthread

run: awProtocolTest handshakeTest
	./awProtocolTest
	./handshakeTest

clean:
	rm -f awProtocolTest handshakeTest

.PHONY: all run clean
