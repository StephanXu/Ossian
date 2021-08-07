
#include <mimalloc.h>
#ifdef ENABLE_GPERF
#include <gperftools/profiler.h>
#endif

#include "ossian/Dispatcher.hpp"
#include "ossian/Pipeline.hpp"

#include <thread>

namespace ossian
{
Dispatcher::Dispatcher(DI::Injector&& injector)
	: m_Injector(std::move(injector))
{
}

void Dispatcher::Run() const
{
	auto executables = m_Injector.GetInstance<DI::ServiceCollection<IExecutable>>();
	std::vector<std::thread> threads;
	for (auto&& item : *executables)
	{
		threads.emplace_back([item]()
		{
#ifdef ENABLE_GPERF
			ProfilerRegisterThread();
#endif
			item->ExecuteProc();
		});
	}
	for (auto&& item : threads)
	{
		item.join();
	}
}
} // namespace ossian
