#include <httplib.h>
#include <gflags/gflags.h>
#include <fmt/format.h>

#include <iostream>
#include <fstream>

DEFINE_string(server, "ossian.mrxzh.com", "The server to fetch data.");
DEFINE_string(id, "5fc0bc6866fce000016bc49c", "The id to fetch data.");
DEFINE_string(dest, "NVConfig.json", "Path to save configuration file.");

int main(int argc, char** argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	std::cout << "Fetch configuration from: " << FLAGS_server << std::endl;

	httplib::Client cli(FLAGS_server.c_str(), 80);
	auto res = cli.Get(fmt::format("/api/argument/{}?pt=true", FLAGS_id).c_str());

	std::ofstream ofs(FLAGS_dest);
	ofs << res->body;

	std::cout << "Configuration saved: " << FLAGS_dest << std::endl;
}
