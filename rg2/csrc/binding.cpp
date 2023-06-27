#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "include/WalkerEnv.hpp"

namespace py = pybind11;

int THREAD_COUNT = 1;

PYBIND11_MODULE(_rg2, m)
{
  py::class_<WalkerEnv>(m, "WalkerEnv")
      .def(py::init<std::string, bool>(), py::arg("resourceDir"), py::arg("visualizable"))
      .def("init", &WalkerEnv::init)
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