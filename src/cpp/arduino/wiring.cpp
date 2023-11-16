#include <chrono>

// millis function implementation
static auto start = std::chrono::high_resolution_clock::now();

unsigned long millis()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
    return static_cast<unsigned long>(duration.count());
}