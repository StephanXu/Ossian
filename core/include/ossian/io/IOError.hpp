#ifndef OSSIAN_CORE_IOERROR
#define OSSIAN_CORE_IOERROR

#ifdef __linux

class IOError : public std::runtime_error
{
public:
    IOError(std::string message) : std::runtime_error(message)
    {
    }
};

#endif // __linux

#endif // OSSIAN_CORE_IOERROR