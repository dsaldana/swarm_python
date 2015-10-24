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





static PyMethodDef EmbMethods[] = {
//    {"numargs", emb_numargs, METH_VARARGS, "Return the number of arguments received by the process."},
//    {"multipli", emb_multipli, METH_VARARGS, "Multiplies in c++."},
        {NULL, NULL, 0, NULL}
};


//////////////////////////////////////////////////
TeamControllerPlugin::TeamControllerPlugin()
        : RobotPlugin() {
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Load(sdf::ElementPtr _sdf) {

  //Py_SetProgramName(3);  /* optional but recommended */
  // Initilize python interpreter
  Py_Initialize();

  /////
  Py_InitModule("robot", EmbMethods);

  //// add the current folder to the workspace
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\".\")");
  PyRun_SimpleString("print '---Swarm-python driver--'");


  pName = PyString_FromString("controller");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);


  // FIXME move this to the finalize method in the driver.
  //Py_Finalize();
}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info) {

  // Error if there is no python script
  if (pModule == NULL) {
    PyErr_Print();
    return;
  }

  // obtain function update
  pFunc = PyObject_GetAttrString(pModule, "update");
  // todo validate pFunc is callable

  // Arguments for the update function
  //pArgs = PyTuple_New();
  // Call the python function
//  pValue =
  PyObject_CallObject(pFunc, NULL);


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
