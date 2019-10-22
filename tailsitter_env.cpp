#include <iostream>
#include "tailsitter.h"
using namespace std;

int main(int argc, char **argv) {
	gazebo::setupServer(argc,argv);
	tailsitter * ts = new tailsitter();
	//printf("ok");
	return 0;
}
