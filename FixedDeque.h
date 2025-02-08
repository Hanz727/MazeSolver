#pragma once

#ifdef DEBUG_PRINT_WINDOWS
    #include <iostream>
#else
    #include <Arduino.h>
#endif

template <typename T>
class FixedDeque {
public:
    FixedDeque(size_t capacity) : capacity(capacity), frontIndex(0), backIndex(0), size(0) {
        data = new T[capacity];
    }

    ~FixedDeque() {
        delete[] data;
    }

    void push_back(T value) {
        if (size < capacity) {
            data[backIndex] = value;
            backIndex = (backIndex + 1) % capacity;
            size++;
        }
    }

    void push_front(T value) {
        if (size < capacity) {
            frontIndex = (frontIndex - 1 + capacity) % capacity;
            data[frontIndex] = value;
            size++;
        }
    }

    T pop_back() {
        if (size > 0) {
            backIndex = (backIndex - 1 + capacity) % capacity;
            size--;
            return data[backIndex];
        }
        return -1;
    }


    T pop_front() {
        if (size > 0) {
            T value = data[frontIndex];
            frontIndex = (frontIndex + 1) % capacity;
            size--;
            return value;
        }
        return {};
    }

    size_t get_size() const {
        return size;
    }

    bool is_empty() const {
        return size == 0;
    }

    bool is_full() const {
        return size == capacity;
    }

private:
    T* data;
    size_t capacity;
    size_t frontIndex;
    size_t backIndex;
    size_t size;
};
