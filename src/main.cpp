#include <ossian/ApplicationBuilder.hpp>
#include <csignal>

#ifdef ENABLE_GPERF
#include <gperftools/profiler.h>
#endif

#include "Startup.hpp"

static void sighandler(int sig_no)
{
	std::exit(0);
}


int main()
{
#ifdef ENABLE_GPERF
	ProfilerStart("Ossian.prof");
#endif
	std::signal(SIGTERM, sighandler);
	ossian::ApplicationBuilder()
		.UseStartup<Startup>()
		.Realization()
		.Run();
#ifdef ENABLE_GPERF
	ProfilerStop();
#endif
}
