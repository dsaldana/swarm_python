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


static PyObject *
robot_set_linear_velocity(PyObject *self, PyObject *args) {
//
  int robot_id;
  float x, y, z;
  PyArg_ParseTuple(args, "ifff", &robot_id, &x, &y, &z);

  printf("setvel-%d\n", robot_id);
  //tcplugins[robot_id]->SetLinearVelocity(ignition::math::Vector3d(x, y, z));


  return Py_BuildValue("i", 5);
}

static PyObject*
robot_multipli(PyObject *self, PyObject *args)
{
  int i, j;
  PyArg_ParseTuple(args, "ii", &i, &j);
  return Py_BuildValue("i", i*j);
}



PyMethodDef EmbMethods[] = {
        {"set_linear_velocity", robot_set_linear_velocity, METH_VARARGS, "Multiplies in c++."},
        {"multiply", robot_multipli, METH_VARARGS, "Multiplies in c++."},
        {NULL, NULL, 0, NULL}
};


//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
        : RobotPlugin() {
}

static int initializated_python = false;


//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf) {

  // Initialize python only once!
  if (!initializated_python) {
    ////////////////////////////////////// initialize Python ////////////////
    Py_Initialize();
    // Thread python
//  PyEval_InitThreads();

    //// add the current folder to the workspace
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\".\")");


    /////////////////////////////////// Control methods ////////////////////////
    Py_InitModule("robot", EmbMethods);
    pName = PyString_FromString("controller");
    pModule = PyImport_Import(pName);


    id_robot = 0;
  } else {
    id_robot = 1;
  }
  tcplugins[id_robot] = this;





  /////////////
  printf("Starting robot %d\n", id_robot);
  initializated_python = true;


  PyRun_SimpleString("print '---Swarm-python driver--'");


//  pFunc = PyObject_GetAttrString(pModule, "update");
//  // todo validate pFunc is callable
//
//  // Run update function
//  PyObject_CallObject(pFunc, NULL);
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

  // robot id as argument
  PyObject * pArgs, *pValue;
  pArgs = PyTuple_New(1);
  pValue = PyInt_FromLong(this->id_robot);
  PyTuple_SetItem(pArgs, 0, pValue);

////  // Run update function
  //printf("robot %d\n", this->id_robot);
  PyObject_CallObject(pFunc, pArgs);
////
//


  // Simple example for moving each type of robot.
  switch (this->Type()) {
    case GROUND: {
      //this->SetLinearVelocity(ignition::math::Vector3d(1, 0, 0));
      tcplugins[id_robot]->SetAngularVelocity(ignition::math::Vector3d(0, 0, 0.1));
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
