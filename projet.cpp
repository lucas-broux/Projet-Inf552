#include <iostream>
#include "json.hpp"
#include <fstream>

using json = nlohmann::json;
using namespace std;

int main()
{
	cout << "Hello World" << endl;

	// read a JSON file
	std::ifstream stream_camera("../Files/aachen_000029_000019_test/aachen_000029_000019_camera.json");
	json camera;
	stream_camera >> camera;
	cout << camera.dump() << endl;

	while (true);
	return 0;
}