#include <httplib.h>
#include <gflags/gflags.h>

#include <iostream>
#include <fstream>

DEFINE_string(server, "ossian.mrxzh.com", "The server to fetch data.");
DEFINE_string(dest, "NVConfig.json", "Path to save configuration file.");

int main(int argc, char **argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);

	std::cout << "Fetch configuration from: " << FLAGS_server << std::endl;

	httplib::Client cli(FLAGS_server.c_str());
	auto res = cli.Get("/api/argument");
	std::ofstream ofs(FLAGS_dest);
	ofs << res->body;
	
	std::cout << "Configuration saved: " << FLAGS_dest << std::endl;
}