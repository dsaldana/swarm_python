/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <sdf/sdf.hh>
#include "TeamControllerPlugin.hh"

//
#include <Python.h>


using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(TeamControllerPlugin)

///////////// python Wrapper //////////////////////
PyObject *pName, *pModule, *pDict, *pFunc;

TeamControllerPlugin *tcplugin;

PyObject *
robot_set_linear_velocity(PyObject *self, PyObject *args) {
  printf("-%d", tcplugin->id_robot);
  float x, y, z;
  PyArg_ParseTuple(args, "fff", &x, &y, &z);

  //tcplugin->SetLinearVelocity(ignition::math::Vector3d(x, y, z));
  //if(!PyArg_ParseTuple(args, ":numargs"))
  //    return NULL;
  return Py_BuildValue("i", 5);
}

PyMethodDef EmbMethods[] = {
        {"set_linear_velocity", robot_set_linear_velocity, METH_VARARGS, "Multiplies in c++."},
        {NULL, NULL, 0, NULL}
};


//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
        : RobotPlugin() {
}

static int initializated_python = false;


void init_python() {
  // initialize Python
  Py_Initialize();

  // Thread python
  PyEval_InitThreads();

  ///// Control methods //////
  Py_InitModule("robot", EmbMethods);

  //// add the current folder to the workspace
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\".\")");

  pName = PyString_FromString("controller");
  pModule = PyImport_Import(pName);
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf) {
  tcplugin = this;

  // Initialize python only once!
  if (!initializated_python) {
    init_python();
    id_robot = 0;
  } else {
    init_python();
    id_robot = 2;
  }
  printf("Starting robot %d\n", id_robot);
  initializated_python = true;


  PyRun_SimpleString("print '---Swarm-python driver--'");


  pFunc = PyObject_GetAttrString(pModule, "update");
//  // todo validate pFunc is callable
//
//  // Run update function
  PyObject_CallObject(pFunc, NULL);
//


  //Py_Finalize();


//  // initialize Python
//  Py_Initialize();
//// initialize thread support
//  PyEval_InitThreads();
////
//////// init thread python
////  PyThreadState * mainThreadState = NULL;
////// save a pointer to the main PyThreadState object
////  mainThreadState = PyThreadState_Get();
//
////
////
//  // get the global lock
////  PyEval_AcquireLock();
//// get a reference to the PyInterpreterState
////  PyInterpreterState *mainInterpreterState = mainThreadState->interp;
////// create a thread state object for this thread
////  PyThreadState *myThreadState = PyThreadState_New(mainInterpreterState);
//
//
//  /////
//  Py_InitModule("robot", EmbMethods);
//
//  //// add the current folder to the workspace
//  PyRun_SimpleString("import sys");
//  PyRun_SimpleString("sys.path.append(\".\")");
//  PyRun_SimpleString("print '---Swarm-python driver--'");
//
//
//  pName = PyString_FromString("controller");
//  pModule = PyImport_Import(pName);
////  Py_DECREF(pName);
//
//
//// free the lock
//  PyEval_ReleaseLock();
//
////  PyRun_SimpleString("import robot");
////  PyRun_SimpleString("robot.set_linear_velocity(1,0,0)");
//
//  // FIXME move this to the finalize method in the driver.
////  Py_Finalize();
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info) {

  // Error if there is no python script
  if (pModule == NULL) {
    PyErr_Print();
    return;
  }

//  // obtain function update
  pFunc = PyObject_GetAttrString(pModule, "update");
////  // todo validate pFunc is callable
////
////  // Run update function
  printf("Starting robot %d\n", id_robot);
  PyObject_CallObject(pFunc, NULL);
////
//


  // Simple example for moving each type of robot.
  switch (this->Type()) {
    case GROUND: {
      //this->SetLinearVelocity(ignition::math::Vector3d(1, 0, 0));
      this->SetAngularVelocity(ignition::math::Vector3d(0, 0, 0.1));
      break;
    }
    case ROTOR: {
      this->SetLinearVelocity(ignition::math::Vector3d(0, 0, 1));
      this->SetAngularVelocity(ignition::math::Vector3d(0, 0, -0.1));
      break;
    }
    case FIXED_WING: {
      this->SetLinearVelocity(ignition::math::Vector3d(1, 0, 0));
      this->SetAngularVelocity(ignition::math::Vector3d(0, -0.4, 0));
      break;
    }
    default: {
      gzerr << "Unknown vehicle type[" << this->Type() << "]\n";
      break;
    }
  };
}
