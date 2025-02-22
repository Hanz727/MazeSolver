#pragma once

#ifdef DEBUG
    #include <iostream>
    #include <stdint.h>
    #include <iomanip>
#else
    #include <Arduino.h>
#endif

static bool _assert_handler(const char* expr, const char* file, int line) {
#ifdef DEBUG
    std::cout << "Assertion failed: " << expr << " at: " << file << ":" << line << "\n";
#else
    Serial.println("Assertion failed: " + String(expr) + " at: " + String(file) + ":" + String(line));
#endif
    return 0;
}

#define assert(expr) ((expr) ? 1 : _assert_handler(#expr, __FILE__, __LINE__))

