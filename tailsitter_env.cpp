#include <iostream>
#include "tailsitter.h"
using namespace std;

int main(int argc, char **argv) {
	gazebo::setupServer(argc,argv);
	tailsitter * ts = new tailsitter();
	for (int i =0 ;i < 100;i++) {
		ts->run_world();
	}
	//printf("ok");
	return 0;
}
