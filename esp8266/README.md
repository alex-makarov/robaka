Robaka is connected ESP-01 board, based on ESP8266. This board can be used to access Arduino remotely via WiFI. One option is to forward the Serial port to the TCP connection, either incoming or outgoing. This folder contains two simple implementations of UART-TCP proxy by Hristo Gochkov, from his excellent ESP8266WiFi library.

There's no build env for these scripts [yet], they are just built in PIO and flashed directly to ESP-01.
