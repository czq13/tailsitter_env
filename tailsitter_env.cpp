#include <iostream>
#include "tailsitter.h"
using namespace std;

#ifdef FORPY
#include <Python.h>
#include <vector>
#include <string>
tailsitter * ts;
float yaw;
int cnt;
static PyObject* test(PyObject *self, PyObject *args) {
    cout << "hello ecl 1" << endl;
    return PyLong_FromLong(0);
}
static PyObject* ResetEnv(PyObject *self, PyObject *args) {
	float vx,vy,vz,roll,pitch,yaw;
	bool ok = PyArg_ParseTuple(args,"ffffff",&vx,&vy,&vz,&roll,&pitch,&yaw);
	ignition::math::Pose3d pose;
	pose.Set(0.0,0.0,0.0,roll,pitch,yaw);
	ts->model->SetWorldPose(pose);
	ignition::math::Vector3d vel;
	vel[0] = vx;
	vel[1] = vy;
	vel[2] = vz;
	ts->model->SetLinearVel(vel);
	ts->thrust_sp = 0.78;
	ts->_v_att_sp.pitch_body = 0.0;
	ts->reset_ctrl();
	yaw = ts->yaw;

	fclose(ts->logfile);
	cnt++;
	if (cnt > 10) cnt = cnt % 10;
	string filename = "log";
	//printf("cnt=%d\n",cnt);
	//cout << char('0' + cnt % 10);
	filename += char('0' + cnt % 10);
	filename += char('0' + cnt / 10);
	filename += ".txt";
	fopen(filename.c_str(),"w");
	if (ok) return Py_BuildValue("i",1);
	else return Py_BuildValue("i",0);
}
static PyObject* SetCtrl(PyObject *self, PyObject *args) {
	float thrust,pitch;
	bool ok = PyArg_ParseTuple(args,"ff",&thrust,&pitch);
	//ts->thrust_sp = thrust;
	//ts->_v_att_sp.pitch_body = pitch;
	ts->thrust_sp = ts->thrust_sp * 0.95 + 0.05 * thrust;
	if (ts->thrust_sp < 0.3) ts->thrust_sp = 0.3;
	ts->_v_att_sp.pitch_body = ts->_v_att_sp.pitch_body * 0.95 + 0.05 * pitch;
	ts->_v_att_sp.roll_body = 0.0;
	ts->_v_att_sp.yaw_body = yaw;
	matrix::Quatf q_sp = matrix::Eulerf(ts->_v_att_sp.roll_body, ts->_v_att_sp.pitch_body, ts->_v_att_sp.yaw_body);
	q_sp.copyTo(ts->_v_att_sp.q_d);
	ts->run_world();
	if (ok) return Py_BuildValue("i",1);
	else return Py_BuildValue("i",0);
}
static PyObject* GetObsv(PyObject *self, PyObject *args) {
	float vx,vz,pitch,roll,pitchspeed,z;
	vx = ts->_local_pos.vx;//ts->vxb;
	vz = ts->_local_pos.vz;//ts->vzb;
	pitch = ts->pitch;
	roll = ts->roll;
	pitchspeed = ts->_v_att.pitchspeed;
	z = ts->_local_pos.z;
	return Py_BuildValue("(ffffff)",vx,vz,pitch,roll,pitchspeed,z);
}

static PyObject* close(PyObject *self, PyObject *args) {
	gazebo::shutdown();
	return PyLong_FromLong(0);
}
// Exported methods are collected in a table
PyMethodDef method_table[] = {
    {"test", (PyCFunction) test, METH_VARARGS, "Method docstring"},
	{"ResetEnv", (PyCFunction) ResetEnv, METH_VARARGS, "reset env"},
	{"SetCtrl", (PyCFunction) SetCtrl, METH_VARARGS, "set ctrl value"},
	{"GetObsv", (PyCFunction) GetObsv, METH_VARARGS, "get observ"},
	{"close", (PyCFunction) close, METH_VARARGS, "close the environment"},
    {NULL, NULL, 0, NULL} // Sentinel value ending the table
};

// A struct contains the definition of a module
PyModuleDef tailsitter_env_module = {
    PyModuleDef_HEAD_INIT,
    "tailsitter_env", // Module name
    "This is the module docstring",
    -1,   // Optional size of the module state memory
    method_table,
    NULL, // Optional slot definitions
    NULL, // Optional traversal function
    NULL, // Optional clear function
    NULL  // Optional module deallocation function
};

// The module init function
PyMODINIT_FUNC PyInit_tailsitter_env(void) {
	bool ok = true;
	ok = gazebo::setupServer();
	ts = new tailsitter();
    return PyModule_Create(&tailsitter_env_module);
}
#else
int main(int argc, char **argv) {
	gazebo::setupServer(argc,argv);
	tailsitter * ts = new tailsitter();
	//ts->init_fw_state();
	double yaw = ts->yaw;
	for (int i =0 ;i < 1000;i++) {
		double t = (ts->world->SimTime()).Double();
		t = math::constrain(t,0.0,3.0);
		ts->_v_att_sp.pitch_body = 0.0f - t * 30.0f / 57.3;
		ts->_v_att_sp.roll_body = 0.0f;
		ts->_v_att_sp.yaw_body = yaw;
		matrix::Quatf q_sp = matrix::Eulerf(ts->_v_att_sp.roll_body, ts->_v_att_sp.pitch_body, ts->_v_att_sp.yaw_body);
		q_sp.copyTo(ts->_v_att_sp.q_d);

		ts->thrust_sp = 0.80;
		ts->roll_weight = 1.0;
		ts->pitch_weight = 0.5;
		ts->yaw_weight = 1.0;
		ts->run_world();
		ignition::math::Vector3d angaccel = ts->model2->WorldAngularAccel();
		printf("%f %f %f\n",angaccel[0],angaccel[1],angaccel[2]);
	}
	//printf("ok");
	return 0;
}
#endif
