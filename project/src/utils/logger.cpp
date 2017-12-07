#include "logger.hpp"


logger::logger(string target) {
	// Open both file streams.
	(file_stream_console).open(target + "_console.txt");
	(file_stream_python).open(target + "_python.txt");
};


void logger::close() {
	// Close both file streams.
	(file_stream_console).close();
	(file_stream_python).close();
};


void logger::log_comment(string cmt) {
	file_stream_console << cmt << endl;
	cout << cmt << endl;
};


void logger::log_message(message msg) {
	file_stream_python << msg.to_python_dict() << endl;
	file_stream_console << msg.to_string() << endl;
	cout << msg.to_string() << endl;
};