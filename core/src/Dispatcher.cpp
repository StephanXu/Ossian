#include <spdlog/spdlog.h>
#include <mimalloc.h>

#include "ossian/Dispatcher.hpp"
#include "ossian/Pipeline.hpp"

#include <chrono>
#include <iostream>

namespace ossian
{

Dispatcher::Dispatcher(DI::Injector &&injector)
    : m_Injector(std::move(injector))
{
}

void Dispatcher::Run()
{
    auto executables = m_Injector.GetInstance<DI::ServiceCollection<IExecutable>>();
    std::vector<std::thread> threads;
	for (auto&& item : *executables)
	{
        threads.emplace_back([item]() {item->ExecuteProc(); });
	}
	for(auto&& item : threads)
	{
        item.join();
	}
}

} // namespace ossian