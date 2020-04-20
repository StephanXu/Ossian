#include <ossian/ApplicationBuilder.hpp>

#include <memory>

#include "Startup.hpp"

int main()
{
	ossian::ApplicationBuilder()
		.UseStartup<Startup>()
		.Realization()
		.Run();
}
