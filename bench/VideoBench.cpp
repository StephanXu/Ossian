
#include <benchmark/benchmark.h>

#include <cmath>

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


BENCHMARK(BMPow)->Arg(8);
BENCHMARK(BMNativeTime)->Arg(8);