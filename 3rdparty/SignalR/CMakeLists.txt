
find_library(SignalRClient_LIB signalrclient)
find_path(SignalRClient_INCLUDE signalrclient/hub_connection.h)
message("FOUND SignalRClient: ${SignalRClient_LIB}")
message("FOUND SignalRClient Headers: ${SignalRClient_INCLUDE}")

add_library(signalrclient UNKNOWN IMPORTED GLOBAL)
set_property(TARGET signalrclient PROPERTY IMPORTED_LOCATION "${SignalRClient_LIB}")
set_property(TARGET signalrclient PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${SignalRClient_INCLUDE}")
set_property(TARGET signalrclient PROPERTY INTERFACE_COMPILE_OPTIONS "-DNO_SIGNALRCLIENT_EXPORTS")