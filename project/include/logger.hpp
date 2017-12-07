#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

/**
* A helper class to implement messages, which will be used in the logger.
* A comment is a piece of information that will be analysed in python.
*/
class message {
private:
	string tag;	// For logging of the comment in the console.
	int input_nb; // Imput number for the comment.
	string description;	// Description/object of the message.
	double performance;	// Performance obtained.
	string performance_unit; // Unit of performance (ex: seconds).

public:
	/**
	* Constructor for the class.
	*/
	message(string tag, int input_nb, string description, double performance, string performance_unit) {
		this->tag = tag;
		this->input_nb = input_nb;
		this->description = description;
		this->performance = performance;
		this->performance_unit = performance_unit;
	};
	
	/**
	* Returns stylized string of the message.
	* Ex: "[INPUT 0]:3D Point cloud computed. Performance of the computation: 0.442000 seconds"
	*/
	string to_string() {
		return tag + ": " + description + " computed. Performance of the computation: " + std::to_string(performance) + " " + performance_unit;
	};

	/**
	* Returns string corresponding to information presented as python dict.
	* Ex: "{'INPUT' : '0', 'INFO' : '3D Point cloud', 'PERF' : '0.442000', 'UNIT' : 'seconds'}"
	*/
	string to_python_dict() {
		return "{'INPUT' : '" + std::to_string(input_nb) + "', 'INFO' : '" + description + "', 'PERF' : '" + std::to_string(performance) + "', 'UNIT' : '" + performance_unit + "'}";
	};

};


/**
* A class for logging of the informations in a file.
* We use it for 2 purposes : 
*	- Logging a comment i.e. a string.
*	- Logging a message i.e. a piece of information which will be further analysed in python.
*/
class logger {

private:	
	string target; // Link to file to fill.
	ofstream file_stream_console; // Text file stream for console-like comments.
	ofstream file_stream_python; // Text file stream for interpretable python contents.

public:

	/**
	* Constructor for the class. Open a file stream in the given target.
	*
	* @param target The link to file into which to keep log.
	*/
	logger(string target);


	/**
	* Close the stream.
	*/
	void close();


	/**
	* Log a comment.
	*
	* @param cmt The comment to log.
	*/
	void log_comment(string cmt);


	/**
	* Log a message.
	*
	* @param msg The message to log.
	*/
	void log_message(message msg);

};