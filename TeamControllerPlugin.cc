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
 * Python function for: Set angular velocity
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

/**
 * Python function for: ask for Neighbors.
 */
static PyObject *
robot_neighbors(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Send to the right controller.
  const std::vector<std::string> &neighbors = tcplugins[robot_id]->Neighbors();

  PyObject * pArgs = PyTuple_New(neighbors.size());

  for (int i = 0; i < neighbors.size(); ++i) {
    PyObject * pValue = Py_BuildValue("s", neighbors.at(i).c_str());

    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, i, pValue);
  }


  return pArgs;
}


/**
 * Python function for: ask for GPS localization.
 */
static PyObject *
robot_pose(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get pose and altitude.
  double latitude, longitude, altitude;
  tcplugins[robot_id]->Pose(latitude, longitude, altitude);

  PyObject * pArgs = PyTuple_New(3);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", latitude));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", longitude));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", altitude));


  return pArgs;
}


/**
 * Python function for: SearchArea function.
 */
static PyObject *
robot_search_area(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get pose and altitude.
  double minLatitude, maxLatitude, minLongitude, maxLongitude;
  tcplugins[robot_id]->SearchArea(minLatitude, maxLatitude, minLongitude, maxLongitude);

  PyObject * pArgs = PyTuple_New(4);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", minLatitude));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", maxLatitude));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", minLongitude));
  PyTuple_SetItem(pArgs, 3, Py_BuildValue("d", maxLongitude));

  return pArgs;
}


/**
 * Python function for: SearchArea function.
 */
static PyObject *
robot_camera(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  // Get the camera information
  ImageData img;
  if (tcplugins[robot_id]->Image(img)) {
    PyObject * camera_locs = PyTuple_New(img.objects.size());
    int i = 0;
    for (auto const obj : img.objects) {
      PyObject * camera_obj = PyTuple_New(1);
      PyTuple_SetItem(camera_obj, 0, Py_BuildValue("s", obj.first.c_str()));
      // TODO obtain pose
      double x, y, z, p, r, ya;
//      obj.second.Pose3(x, y, z, p, r, ya);
//      PyTuple_SetItem(camera_obj, 1, Py_BuildValue("d", x));
//      PyTuple_SetItem(camera_obj, 2, Py_BuildValue("d", y));
//      PyTuple_SetItem(camera_obj, 3, Py_BuildValue("d", z));
//      PyTuple_SetItem(camera_obj, 4, Py_BuildValue("d", p));
//      PyTuple_SetItem(camera_obj, 5, Py_BuildValue("d", r));
//      PyTuple_SetItem(camera_obj, 6, Py_BuildValue("d", ya));
//

      PyTuple_SetItem(camera_locs, i++, camera_obj);
    }
    return camera_locs;
  }


  return Py_BuildValue("i", -1);
}


/**
 * Python methods.
 */
static PyMethodDef EmbMethods[] = {
        {"set_linear_velocity",  robot_set_linear_velocity,  METH_VARARGS, "Linear velocity."},
        {"set_angular_velocity", robot_set_angular_velocity, METH_VARARGS, "Angular velocity."},
        {"neighbors",            robot_neighbors,            METH_VARARGS, "Neighbors."},
        {"pose",                 robot_pose,                 METH_VARARGS, "Robot pose using GPS."},
        {"search_area",          robot_search_area,          METH_VARARGS, "Search area for GPS."},
        {"camera",               robot_camera,               METH_VARARGS, "Search area for GPS."},
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

//  const std::vector<std::string> &vector1 = Neighbors();
//
//  for(int i=0; i<vector1.size(); i++){
//    std::cout << vector1.at(i)<<"\n";
//  }

//  printf("--------------");


}
