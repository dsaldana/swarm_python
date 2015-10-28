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
TeamControllerPlugin *tcplugins[1000];


/**
 * Python function for: Set linear velocity
 */
static PyObject *
robot_set_linear_velocity(PyObject *self, PyObject *args) {
  int robot_id;
  float x, y, z;
  PyArg_ParseTuple(args, "ifff", &robot_id, &x, &y, &z);

  // Send to the right controller.
  tcplugins[robot_id]->SetLinearVelocity(ignition::math::Vector3d(x, y, z));
  return Py_BuildValue("i", robot_id);
}

/**
 * Python function for: Set linear velocity
 */
static PyObject *
robot_set_angular_velocity(PyObject *self, PyObject *args) {
  int robot_id;
  float x, y, z;
  PyArg_ParseTuple(args, "ifff", &robot_id, &x, &y, &z);

  // Send to the right controller.
  tcplugins[robot_id]->SetAngularVelocity(ignition::math::Vector3d(x, y, z));
  return Py_BuildValue("i", robot_id);
}



static PyMethodDef EmbMethods[] = {
        {"set_linear_velocity", robot_set_linear_velocity, METH_VARARGS, "Linear velocity."},
        {"set_angular_velocity", robot_set_angular_velocity, METH_VARARGS, "Angular velocity."},
        {NULL, NULL, 0, NULL}
};


static int initializated_python = false;
static int robot_counter = 0;
//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
        : RobotPlugin() {

  id_robot = robot_counter;
  robot_counter++;
  if (!initializated_python) {
    initializated_python = true;
  }

  tcplugins[id_robot] = this;
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf) {
  // Initialize python interpreter
  Py_Initialize();

  /////
  Py_InitModule("robot", EmbMethods);

  //// add the current folder to the workspace
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\".\")");
  PyRun_SimpleString("print '---Swarm-python driver--'");


  // Import controller.py
  pName = PyString_FromString("controller");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info) {

  // Error if there is no python script
  if (pModule == NULL) {
    PyErr_Print();
    return;
  }

  // Obtain function update
  pFunc = PyObject_GetAttrString(pModule, "update");
  // todo validate pFunc is callable


  // Robot id as argument
  PyObject * pArgs, *pValue;
  pArgs = PyTuple_New(1);
  pValue = PyInt_FromLong(this->id_robot);
  PyTuple_SetItem(pArgs, 0, pValue);

  // Call UPDATE function in python.
  PyObject_CallObject(pFunc, pArgs);
}
