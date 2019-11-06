#include <iostream>
#include "tailsitter.h"
using namespace std;

int main(int argc, char **argv) {
	gazebo::setupServer(argc,argv);
	tailsitter * ts = new tailsitter();
	ts->init_fw_state();
	for (int i =0 ;i < 10000;i++) {
		ts->run_world();
	}
	//printf("ok");
	return 0;
}
