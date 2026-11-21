// Minimal Arduino.h for the host-side unit tests (pio test -e native): the types and
// functions the control library uses, nothing that touches the hardware.
#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>

// FreeRTOS types that config.h uses for the task priorities and cores
typedef unsigned int UBaseType_t;
typedef int BaseType_t;

// Serial: the logs go nowhere (they are compiled in only when USE_MICROROS is 0)
struct HostSerial
{
    void println(const char *) {}
    template <typename... Args>
    void printf(const char *, Args...) {}
};
[[maybe_unused]] static HostSerial Serial;
