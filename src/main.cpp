#include <ossian/ossian.hpp>

#include <memory>

#include "Startup.hpp"

std::unique_ptr<ossian::ApplicationBuilder> CreateApplicationBuilder()
{
    auto builder = std::make_unique<ossian::ApplicationBuilder>();
    Startup::ConfigServices(*builder);
    Startup::ConfigPipeline(*builder);
    return builder;
}

int main()
{
    CreateApplicationBuilder()->Realization().Run();
}