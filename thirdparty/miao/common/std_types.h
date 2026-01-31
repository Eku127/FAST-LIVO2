//
// Standard types for MIAO optimizer
//

#ifndef MIAO_STD_TYPES_H
#define MIAO_STD_TYPES_H

#include <mutex>
#include <thread>

namespace lightning {

using UL = std::unique_lock<std::mutex>;

}  // namespace lightning

#endif  // MIAO_STD_TYPES_H
