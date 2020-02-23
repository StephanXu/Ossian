#include "ossian/io/CAN.hpp"
#include "ossian/io/UART.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/io/IO.hpp"
#include <iostream>
#include <memory>
#include <vector>
using namespace ossian;
using namespace std;

void func(std::shared_ptr<BaseDevice> device,
	size_t length,
	std::shared_ptr<uint8_t[]> data)
{
	cout << length << endl;
}

int main()
{
	auto mgr = std::make_shared<CANManager>();
	mgr->AddDevice("can0",0x100, func);
	std::vector<BaseHardwareManager*> managers = { mgr.get() };
	IOListener listener(&managers);
	while (true)
	{
		listener.Listen(1000);
	}
	cout << "hello" << endl;
	return 0;
}