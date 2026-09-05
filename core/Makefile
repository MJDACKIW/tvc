# Builds core/ as a shared library for the Python simulation (ctypes) and for
# tools/parity_test.py. See SPEC.md Section 1 (rationale) and Section 6.1 (parity test).
#
# -O0 -ffp-contract=off are pinned here AND in firmware/platformio.ini's [env:native] so the
# same source, compiled two ways, produces bitwise-identical float32 arithmetic; otherwise a
# parity mismatch could come from codegen differences instead of a real controller_step bug.

UNAME_S := $(shell uname -s)
# Make's built-in default for CXX is g++, which defeats a plain `?=`, so detect that
# default explicitly and fall back to clang++ per SPEC.md Section 0/6. An explicit
# `make CXX=...` or an exported CXX in the environment still wins either way.
ifeq ($(origin CXX),default)
CXX = clang++
endif
CXXFLAGS = -std=c++17 -O0 -ffp-contract=off -fPIC -Wall -Wextra
SOURCES = kalman1d.cpp pid.cpp rate_limiter.cpp controller.cpp ffi.cpp

ifeq ($(UNAME_S),Darwin)
	TARGET = libtvccore.dylib
	LDFLAGS = -dynamiclib
else
	TARGET = libtvccore.so
	LDFLAGS = -shared
endif

.PHONY: native clean
native: $(TARGET)

$(TARGET): $(SOURCES) $(wildcard *.h)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(SOURCES)

clean:
	rm -f libtvccore.dylib libtvccore.so
