#include <gflags/gflags.h>
#include <fmt/format.h>

#include <iostream>
#include <fstream>

#include "../src/OnlineDebug.hpp"

int main(int argc, char** argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	const std::string demoArgumentId{"606b1215cf72a100012f92b7"};
	OnlineDebug onlineDbg;
	onlineDbg.Connect("http://debug.fenzhengrou.wang:14003/logger");
	onlineDbg.UploadOfflineLog("offline-log.log",
	                           "OnlineDebuggerTest",
	                           "Test of online debugger",
	                           demoArgumentId);
}
