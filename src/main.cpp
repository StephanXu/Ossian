#include <ossian/ApplicationBuilder.hpp>

#include "Startup.hpp"

int main()
{
	ossian::ApplicationBuilder()
		.UseStartup<Startup>()
		.Realization()
		.Run();
}
