
#include <benchmark/benchmark.h>
#include "Config.pb.h"
#include <cmath>
#include <unordered_map>
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
        float result = aimbot->areanormalizedbase()+202;
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

BENCHMARK(BMProtobufParam);
BENCHMARK(BMNoProtobufParam);

BENCHMARK(BMPow)->Arg(8);
BENCHMARK(BMNativeTime)->Arg(8);

BENCHMARK(BMIntHashMap);
BENCHMARK(BMStringHashMap);