#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "include/WalkerEnv.hpp"

namespace py = pybind11;

int THREAD_COUNT = 1;

PYBIND11_MODULE(_rg2, m)
{
  py::class_<WalkerEnvConfig>(m, "CWalkerEnvConfig")
      .def(py::init<py::dict>());

  py::class_<WalkerEnv>(m, "CWalkerEnv")
      .def(py::init<const WalkerEnvConfig &, bool>())
      .def("reset", &WalkerEnv::reset)
      .def("observe", &WalkerEnv::observe)
      .def("step", &WalkerEnv::step)
      .def("setSeed", &WalkerEnv::setSeed)
      .def("close", &WalkerEnv::close)
      .def("isTerminalState", &WalkerEnv::isTerminalState)
      .def("setSimulationTimeStep", &WalkerEnv::setSimulationTimeStep)
      .def("setControlTimeStep", &WalkerEnv::setControlTimeStep)
      .def("getObDim", &WalkerEnv::getObDim)
      .def("getActionDim", &WalkerEnv::getActionDim)
      .def("turnOnVisualization", &WalkerEnv::turnOnVisualization)
      .def("turnOffVisualization", &WalkerEnv::turnOffVisualization)
      .def("stopRecordingVideo", &WalkerEnv::stopRecordingVideo)
      .def("startRecordingVideo", &WalkerEnv::startRecordingVideo)
      .def("curriculumUpdate", &WalkerEnv::curriculumUpdate);
}