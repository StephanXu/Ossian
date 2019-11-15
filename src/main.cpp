#include <nv/nv.hpp>

#include <memory>

#include "Startup.hpp"

std::unique_ptr<NautilusVision::IOAP::ApplicationBuilder> CreateApplicationBuilder()
{
    auto builder = std::make_unique<NautilusVision::IOAP::ApplicationBuilder>();
    Startup::ConfigServices(*builder);
    Startup::ConfigPipeline(*builder);
    return builder;
}

int main()
{
    CreateApplicationBuilder()->Realization().Run();
}