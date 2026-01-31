//
// Timer utility for MIAO (simplified version)
// Provides compatible interface with lightning-lm Timer
//

#pragma once

#include <chrono>
#include <functional>
#include <string>

namespace lightning {

/// Timer utility for profiling (simplified)
class Timer {
public:
    /**
     * Evaluate and optionally print function execution time
     */
    template <class F>
    static void Evaluate(F&& func, const std::string& func_name, bool print = false) {
        auto t1 = std::chrono::steady_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::steady_clock::now();
        
        if (print) {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            // Simple print, no glog dependency
            (void)func_name;
            (void)time_used;
        }
    }

    /// Print all recorded times (no-op in simplified version)
    static void PrintAll() {}

    /// Dump into file (no-op in simplified version)
    static void DumpIntoFile(const std::string& /*file_name*/) {}

    /// Get mean time (returns 0 in simplified version)
    static double GetMeanTime(const std::string& /*func_name*/) { return 0.0; }

    /// Clear records (no-op in simplified version)
    static void Clear() {}
};

}  // namespace lightning

// Also provide miao namespace alias for compatibility
namespace miao = lightning;
