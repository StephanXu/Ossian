#include "ossian/Utility.hpp"

namespace ossian
{
auto ConvertEndian(const uint16_t x) -> uint16_t
{
	uint16_t res{};
	res |= (x >> 8) & 0x00ff;
	res |= (x << 8) & 0xff00;
	return res;
}
} // ossian
