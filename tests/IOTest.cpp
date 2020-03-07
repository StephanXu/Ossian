#include "ossian/io/CAN.hpp"
#include "ossian/io/UART.hpp"
#include "ossian/IOListener.hpp"
#include "ossian/io/IO.hpp"
#include <iostream>
#include <memory>
#include <vector>
using namespace ossian;
using namespace std;

void func(const BaseDevice* device,
	const size_t length,
	const uint8_t* data)
{
	cout << length << endl;
}

int main()
{
	auto mgr = std::make_shared<CANManager>();
	mgr->AddDevice("can0",0x100)->SetCallback(func);
	auto buses = mgr->GetBuses();
	std::vector<IListenable*> lis;
	for (auto bus : buses)
	{
		lis.push_back(bus);
	}
	IOListener listener(&lis);
	while (true)
	{
		listener.Listen(1000);
	}
	return 0;
}