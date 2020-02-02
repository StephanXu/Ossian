#ifndef OSSIAN_CORE_IO_TYPES
#define OSSIAN_CORE_IO_TYPES

#include <memory>
#include <tuple>

namespace ossian
{

using ReceiveCallback = void(
    size_t length,
    std::shared_ptr<uint8_t[]> data);

using FileDescriptor = int;
using FrameData = std::tuple<size_t, std::shared_ptr<uint8_t[]>>;

enum class IOType
{
    UART,
    CAN
};

} // ossian

#endif // OSSIAN_CORE_IO_TYPES