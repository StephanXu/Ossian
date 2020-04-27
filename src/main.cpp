#include <ossian/ApplicationBuilder.hpp>
#include <csignal>
#include <gperftools/profiler.h>

#include "Startup.hpp"

static void sighandler(int sig_no)
{
	std::exit(0);
}


int main()
{
	ProfilerStart("Ossian.prof");
	std::signal(SIGTERM, sighandler);
	ossian::ApplicationBuilder()
		.UseStartup<Startup>()
		.Realization()
		.Run();
	ProfilerStop();
}
