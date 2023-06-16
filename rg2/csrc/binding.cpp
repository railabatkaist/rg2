#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Env.cpp"

namespace py = pybind11;

int THREAD_COUNT = 1;

PYBIND11_MODULE(_rg2, m)
{
  py::class_<UnitEnv>(m, "UnitEnv")
      .def(py::init<std::string, bool>(), py::arg("resourceDir"), py::arg("visualizable"))
      .def("init", &UnitEnv::init)
      .def("reset", &UnitEnv::reset)
      .def("observe", &UnitEnv::observe)
      .def("step", &UnitEnv::step)
      .def("setSeed", &UnitEnv::setSeed)
      .def("close", &UnitEnv::close)
      .def("isTerminalState", &UnitEnv::isTerminalState)
      .def("setSimulationTimeStep", &UnitEnv::setSimulationTimeStep)
      .def("setControlTimeStep", &UnitEnv::setControlTimeStep)
      .def("getObDim", &UnitEnv::getObDim)
      .def("getActionDim", &UnitEnv::getActionDim)
      .def("turnOnVisualization", &UnitEnv::turnOnVisualization)
      .def("turnOffVisualization", &UnitEnv::turnOffVisualization)
      .def("stopRecordingVideo", &UnitEnv::stopRecordingVideo)
      .def("startRecordingVideo", &UnitEnv::startRecordingVideo)
      .def("curriculumUpdate", &UnitEnv::curriculumUpdate);
}