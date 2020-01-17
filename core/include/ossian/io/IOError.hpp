#ifndef NAUTILUS_VISION_DEVICE_IOERROR
#define NAUTILUS_VISION_DEVICE_IOERROR
class IOError :public std::runtime_error
{
public:
	IOError(std::string message) :std::runtime_error(message) {}
};
#endif