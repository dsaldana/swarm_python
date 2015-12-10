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

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Model.hh"
#include <sdf/sdf.hh>
#include "TeamControllerPlugin.hh"

//
#include <Python.h>


using namespace swarm;

GZ_REGISTER_MODEL_PLUGIN(TeamControllerPlugin)

///////////// python Wrapper //////////////////////
PyObject *pName, *pModule, *pDict, *pUpdateFunc, *pOnDataReceivedFunc;
TeamControllerPlugin *tcplugins[1000];


// For localization MODEL Type
static const char *const MODEL_TYPE = "rotor_";
//static const char *const MODEL_TYPE = "ground_";

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

  PyObject *pArgs = PyTuple_New(neighbors.size());

  for (int i = 0; i < neighbors.size(); ++i) {
    PyObject *pValue = Py_BuildValue("s", neighbors.at(i).c_str());

    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, i, pValue);
  }


  return pArgs;
}

/**
 * Python function for: ask for sending.
 */
static PyObject *
robot_send_to(PyObject *self, PyObject *args) {
  int robot_id;
  char *data, *dest;

  // Get arguments
  PyArg_ParseTuple(args, "iss", &robot_id, &data, &dest);

  // Send message
  std::string sdata(data);
  std::string sdest(dest);
  bool sent = tcplugins[robot_id]->SendTo(sdata, sdest);

  return Py_BuildValue("b", sent);
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

  PyObject *pArgs = PyTuple_New(3);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", latitude));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", longitude));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", altitude));


  return pArgs;
}

/**
 * Python function for: ask for GPS localization.
 */
static PyObject *
robot_gazebo_pose(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);

  //FIXME Do it generic for any model.
  std::string aa = MODEL_TYPE + std::to_string(robot_id);

  gazebo::physics::ModelPtr gazebo_model = gazebo::physics::get_world()->GetModel(aa);
  double rx = gazebo_model->GetWorldPose().pos.x;
  double ry = gazebo_model->GetWorldPose().pos.y;
  double rz = gazebo_model->GetWorldPose().pos.z;
//
//
  double rrx = gazebo_model->GetWorldPose().rot.GetAsEuler().x;
  double rry = gazebo_model->GetWorldPose().rot.GetAsEuler().y;
  double rrz = gazebo_model->GetWorldPose().rot.GetAsEuler().z;
//
//
  PyObject *pArgs = PyTuple_New(6);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("d", rx));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("d", ry));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("d", rz));
  PyTuple_SetItem(pArgs, 3, Py_BuildValue("d", rrx));
  PyTuple_SetItem(pArgs, 4, Py_BuildValue("d", rry));
  PyTuple_SetItem(pArgs, 5, Py_BuildValue("d", rrz));


  return pArgs;
}

/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_imu(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);


  // Get IMU information.
  ignition::math::Vector3d linVel, angVel;
  ignition::math::Quaterniond orient;
  tcplugins[robot_id]->Imu(linVel, angVel, orient);

  // Return
  PyObject *pArgs = PyTuple_New(9);
  PyTuple_SetItem(pArgs, 0, Py_BuildValue("f", linVel.X()));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("f", linVel.Y()));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("f", linVel.Z()));
  PyTuple_SetItem(pArgs, 3, Py_BuildValue("f", angVel.X()));
  PyTuple_SetItem(pArgs, 4, Py_BuildValue("f", angVel.Y()));
  PyTuple_SetItem(pArgs, 5, Py_BuildValue("f", angVel.Z()));
  PyTuple_SetItem(pArgs, 6, Py_BuildValue("f", orient.X()));
  PyTuple_SetItem(pArgs, 7, Py_BuildValue("f", orient.Y()));
  PyTuple_SetItem(pArgs, 8, Py_BuildValue("f", orient.Z()));


  return pArgs;
}


/**
 * Python function for: ask for IMU.
 */
static PyObject *
robot_bearing(PyObject *self, PyObject *args) {
  int robot_id;
  PyArg_ParseTuple(args, "i", &robot_id);


  // Get IMU information.
  ignition::math::Angle bearing;
  tcplugins[robot_id]->Bearing(bearing);

  // Return
  return Py_BuildValue("f", bearing.Radian());
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

  PyObject *pArgs = PyTuple_New(4);
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
    PyObject *camera_locs = PyTuple_New(img.objects.size());
    int i = 0;
    for (auto const obj : img.objects) {
      PyObject *camera_obj = PyTuple_New(1);
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
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzmsg(PyObject *self, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzmsg << message << "\n";
  return Py_BuildValue("i", -1);
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzerr(PyObject *self, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzerr << message << "\n";
  return Py_BuildValue("i", -1);
}

/**
 * Python function for: print gazebo logging messages.
 */
static PyObject *
robot_gzlog(PyObject *self, PyObject *args) {
  int robot_id;
  char *message;

  // Get arguments
  PyArg_ParseTuple(args, "is", &robot_id, &message);

  // Print
  gzlog << message << "\n";
  return Py_BuildValue("i", -1);
}

/**
 * Python methods to call c++.
 */
static PyMethodDef EmbMethods[] = {
        {"set_linear_velocity",  robot_set_linear_velocity,  METH_VARARGS, "Linear velocity."},
        {"set_angular_velocity", robot_set_angular_velocity, METH_VARARGS, "Angular velocity."},
        {"send_to",              robot_send_to,              METH_VARARGS, "Send message to."},
        {"neighbors",            robot_neighbors,            METH_VARARGS, "Neighbors."},
        {"pose",                 robot_pose,                 METH_VARARGS, "Robot pose using GPS."},
        {"imu",                  robot_imu,                  METH_VARARGS, "Robot IMU."},
        {"bearing",              robot_bearing,              METH_VARARGS, "Robot bearing."},
        {"search_area",          robot_search_area,          METH_VARARGS, "Search area for GPS."},
        {"camera",               robot_camera,               METH_VARARGS, "Logic camera."},
        {"gazebo_pose",          robot_gazebo_pose,          METH_VARARGS, "Get pose from gazebo."},
        {"gzmsg",                robot_gzmsg,                METH_VARARGS, "Print gazebo message."},
        {"gzerr",                robot_gzerr,                METH_VARARGS, "Print gazebo error."},
        {"gzlog",                robot_gzlog,                METH_VARARGS, "Gazebo log."},
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
  //////////////////// Python interpreter ////////////////
  Py_Initialize();

  Py_InitModule("robot", EmbMethods);

  //// add the current folder to the workspace
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("sys.path.append(\".\")");
  PyRun_SimpleString("print '---Swarm-python driver--'");


  // Import controller.py
  pName = PyString_FromString("controller");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);


  // Error if there is no python script
  if (pModule == NULL) {
    PyErr_Print();
    return;
  }

  //////////////////// Python functions //////////////////////////////////
  // Python update function.
  pUpdateFunc = PyObject_GetAttrString(pModule, "update");
  // todo validate pUpdateFunc is callable
  // On data received function.
  pOnDataReceivedFunc = PyObject_GetAttrString(pModule, "on_data_received");


  ////////////////////////// Communication ////////////////////////////
  // Read the <num_messages> SDF parameter.
  if (!_sdf->HasElement("num_messages")) {
    gzerr << "TeamControllerPlugin::Load(): Unable to find the <num_messages> "
    << "parameter" << std::endl;
    return;
  }

  // Bind on my local address and default port.
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->Host());

  // Bind on the multicast group and default port.
  this->Bind(&TeamControllerPlugin::OnDataReceived, this, this->kMulticast);

}

//////////////////////////////////////////////////
void TeamControllerPlugin::Update(const gazebo::common::UpdateInfo &_info) {

  // Robot id as argument
  PyObject *pArgs;
  pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, PyInt_FromLong(this->id_robot));

  // Call UPDATE function in python.
  PyObject_CallObject(pUpdateFunc, pArgs);
}


////////////////////////////////////////////////////
void TeamControllerPlugin::OnDataReceived(const std::string &_srcAddress,
                                          const std::string &_dstAddress, const uint32_t _dstPort,
                                          const std::string &_data) {
  // Received arguments
  PyObject *pArgs = PyTuple_New(5);
  PyTuple_SetItem(pArgs, 0, PyInt_FromLong(this->id_robot));
  PyTuple_SetItem(pArgs, 1, Py_BuildValue("s", _srcAddress.c_str()));
  PyTuple_SetItem(pArgs, 2, Py_BuildValue("s", _dstAddress.c_str()));
  PyTuple_SetItem(pArgs, 3, PyInt_FromLong(_dstPort));
  PyTuple_SetItem(pArgs, 4, Py_BuildValue("s", _data.c_str()));


  // Call UPDATE function in python.
  PyObject_CallObject(pOnDataReceivedFunc, pArgs);
}