
#include <benchmark/benchmark.h>
#include "Config.pb.h"
#include <cmath>
#include <unordered_map>
#include <typeindex>

static void BMPow(benchmark::State& state)
{
	for (auto _ : state)
	{
		//std::vector<int> v(state.range(0), state.range(0));
		int result = std::pow(state.range(0), 2);
		//benchmark::DoNotOptimize(v.data());
		//benchmark::ClobberMemory();
	}
}

static void BMNativeTime(benchmark::State& state)
{
	for (auto _ : state)
	{
		//std::vector<int> v(state.range(0), state.range(0));
		int result = state.range(0) * state.range(0);
		//benchmark::DoNotOptimize(v.data());
		//benchmark::ClobberMemory();
	}
}

static void BMProtobufParam(benchmark::State& state)
{
	OssianConfig::Configuration config;
	auto aimbot = config.mutable_aimbot();
	aimbot->set_areanormalizedbase(1.25);
	for (auto _ : state)
	{
		float result = aimbot->areanormalizedbase() + 202;
	}
}

struct config
{
	float aimbot;
};

static void BMNoProtobufParam(benchmark::State& state)
{
	config c;
	c.aimbot = 100;
	for (auto _ : state)
	{
		float result = c.aimbot + 202;
	}
}

static void BMIntHashMap(benchmark::State& state)
{
	std::unordered_map<int, int> map;
	map.insert(std::make_pair(1, 2));
	map.insert(std::make_pair(2, 3));
	map.insert(std::make_pair(3, 4));
	for (auto _ : state)
	{
		float res = map.find(2)->second + 10;
	}
}

static void BMStringHashMap(benchmark::State& state)
{
	std::unordered_map<std::string, int> map;
	map.insert(std::make_pair("1", 2));
	map.insert(std::make_pair("2", 3));
	map.insert(std::make_pair("3", 4));
	for (auto _ : state)
	{
		float res = map.find("2")->second + 10;
	}
}

static void BMSharedPtrHashMap(benchmark::State& state)
{
	std::unordered_map<std::shared_ptr<int>, int> map;
	auto ptr1 = std::make_shared<int>(1);
	auto ptr2 = std::make_shared<int>(2);
	auto ptr3 = std::make_shared<int>(3);
	map.insert(std::make_pair(ptr1, 2));
	map.insert(std::make_pair(ptr2, 3));
	map.insert(std::make_pair(ptr3, 4));
	for (auto _ : state)
	{
		float res = map.find(ptr3)->second + 10;
	}
}

static void BMTypeIndexPtrHashMap(benchmark::State& state)
{
	std::unordered_map<std::type_index, int> map;
	map.insert(std::make_pair(std::type_index(typeid(int)), 2));
	map.insert(std::make_pair(std::type_index(typeid(double)), 3));
	map.insert(std::make_pair(std::type_index(typeid(float)), 4));
	for (auto _ : state)
	{
		float res = map.find(std::type_index(typeid(double)))->second + 10;
	}
}

class IInterface
{
public:
	virtual auto GetStatus(int foo)->int = 0;
};

class AddImpl final :public IInterface
{
public:
	auto GetStatus(int foo)->int override
	{
		return foo + 100;
	}
};

class MinusImpl final :public IInterface
{
public:
	auto GetStatus(int foo)->int override
	{
		return foo - 100;
	}
};

std::unique_ptr<IInterface> CreateImpl(std::string bar)
{
	if (bar.compare("add") == 0)
	{
		return std::make_unique<AddImpl>();
	}
	else
	{
		return std::make_unique<MinusImpl>();
	}
}

struct AddTemplateImpl
{
	auto GetStatus(int foo) { return foo + 100; }
};

struct MinusTemplateImpl
{
	auto GetStatus(int foo) { return foo - 100; }
};

template<class Operation>
class TemplateInterface : public IInterface
{
	Operation m_Operation;
public:
	auto GetStatus(int bar)->int final
	{
		return m_Operation.GetStatus(bar);
	}
};

std::unique_ptr<IInterface> CreateTemplateImpl(std::string bar)
{
	if (bar.compare("add") == 0)
	{
		return std::make_unique<TemplateInterface<AddTemplateImpl>>();
	}
	else
	{
		return std::make_unique<TemplateInterface<MinusTemplateImpl>>();
	}
}

static void BMVirtualFunctionCall(benchmark::State& state)
{
	std::unique_ptr<IInterface> obj = CreateImpl("add");
	for (auto _ : state)
	{
		auto result = obj->GetStatus(1000);
	}
}

static void BMTemplateCall(benchmark::State& state)
{
	std::unique_ptr<IInterface> obj = CreateTemplateImpl("add");
	for (auto _ : state)
	{
		auto result = obj->GetStatus(1000);
	}
}

BENCHMARK(BMProtobufParam);
BENCHMARK(BMNoProtobufParam);

BENCHMARK(BMPow)->Arg(8);
BENCHMARK(BMNativeTime)->Arg(8);

BENCHMARK(BMIntHashMap);
BENCHMARK(BMStringHashMap);
BENCHMARK(BMSharedPtrHashMap);
BENCHMARK(BMTypeIndexPtrHashMap);

BENCHMARK(BMVirtualFunctionCall);
BENCHMARK(BMTemplateCall);