//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Env.cpp"
#include "VecEnvWrapper.hpp"

namespace py = pybind11;
using namespace raisim;
int THREAD_COUNT = 1;

#ifndef ENV_NAME
#define ENV_NAME CVecEnv
#endif

PYBIND11_MODULE(RAISIMGYM_TORCH_ENV_NAME, m)
{
  py::class_<VectorizedEnvironment<UnitEnv>>(m, RSG_MAKE_STR(ENV_NAME))
      .def(py::init<std::string, std::string>(), py::arg("resourceDir"), py::arg("cfg"))
      .def("init", &VectorizedEnvironment<UnitEnv>::init)
      .def("reset", &VectorizedEnvironment<UnitEnv>::reset)
      .def("observe", &VectorizedEnvironment<UnitEnv>::observe)
      .def("step", &VectorizedEnvironment<UnitEnv>::step)
      .def("setSeed", &VectorizedEnvironment<UnitEnv>::setSeed)
      .def("rewardInfo", &VectorizedEnvironment<UnitEnv>::getRewardInfo)
      .def("close", &VectorizedEnvironment<UnitEnv>::close)
      .def("isTerminalState", &VectorizedEnvironment<UnitEnv>::isTerminalState)
      .def("setSimulationTimeStep", &VectorizedEnvironment<UnitEnv>::setSimulationTimeStep)
      .def("setControlTimeStep", &VectorizedEnvironment<UnitEnv>::setControlTimeStep)
      .def("getObDim", &VectorizedEnvironment<UnitEnv>::getObDim)
      .def("getActionDim", &VectorizedEnvironment<UnitEnv>::getActionDim)
      .def("getNumOfEnvs", &VectorizedEnvironment<UnitEnv>::getNumOfEnvs)
      .def("turnOnVisualization", &VectorizedEnvironment<UnitEnv>::turnOnVisualization)
      .def("turnOffVisualization", &VectorizedEnvironment<UnitEnv>::turnOffVisualization)
      .def("stopRecordingVideo", &VectorizedEnvironment<UnitEnv>::stopRecordingVideo)
      .def("startRecordingVideo", &VectorizedEnvironment<UnitEnv>::startRecordingVideo)
      .def("curriculumUpdate", &VectorizedEnvironment<UnitEnv>::curriculumUpdate)
      .def("getObStatistics", &VectorizedEnvironment<UnitEnv>::getObStatistics)
      .def("setObStatistics", &VectorizedEnvironment<UnitEnv>::setObStatistics)
      //.def("setInitConstants", &VectorizedEnvironment<UnitEnv>::setInitConstants)
      .def(py::pickle(
          [](const VectorizedEnvironment<UnitEnv> &p) { // __getstate__ --> Pickling to Python
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(p.getResourceDir(), p.getCfgString());
          },
          [](py::tuple t) { // __setstate__ - Pickling from Python
            if (t.size() != 2)
            {
              throw std::runtime_error("Invalid state!");
            }

            /* Create a new C++ instance */
            VectorizedEnvironment<UnitEnv> p(t[0].cast<std::string>(), t[1].cast<std::string>());

            return p;
          }));

  py::class_<NormalSampler>(m, "NormalSampler")
      .def(py::init<int>(), py::arg("dim"))
      .def("seed", &NormalSampler::seed)
      .def("sample", &NormalSampler::sample);
}