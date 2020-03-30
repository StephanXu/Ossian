#include "../core/include/ossian/ApplicationBuilder.hpp"

class IPrinter
{
public:
	virtual auto PrintText()->void = 0;
};

class HelloPrinter : public IPrinter
{
public:
	OSSIAN_SERVICE_SETUP(HelloPrinter()) {}
	auto PrintText()->void override { spdlog::warn("Hello"); }
};

class WorldPrinter : public IPrinter
{
public:
	OSSIAN_SERVICE_SETUP(WorldPrinter()) {}
	auto PrintText()->void override { spdlog::warn("World"); }
};

class ResultGenerator
{
public:
	OSSIAN_SERVICE_SETUP(ResultGenerator(ossian::DI::ServiceCollection<IPrinter>* loader))
	{
		for (auto&& item : *loader) { item->PrintText(); }
	}
};

class ResultPeeker
{
public:
	OSSIAN_SERVICE_SETUP(ResultPeeker(IPrinter* loader))
	{
		loader->PrintText();
	}
};

int main()
{
	auto app = std::make_unique<ossian::ApplicationBuilder>();
	app->AddService<IPrinter, HelloPrinter>();
	app->AddService<IPrinter, WorldPrinter>();
	app->AddService<ResultGenerator>();
	app->AddService<ResultPeeker>();

	app->Realization();
}