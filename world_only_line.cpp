#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "Env.h"

namespace py = pybind11;

int test_add(int i, int j)
{
    return i * j;
}

PYBIND11_MODULE(world_only_line, m)
{
    m.doc() = "pybind11 test";
    m.def("test_add", &test_add, "A function test doc");
    py::class_<physics::Env>(m, "Env")
        .def(py::init<>())
        .def("reset", &physics::Env::reset)
        .def("set_test_agent", &physics::Env::testEnvSet2)
        .def("step", &physics::Env::step)
        .def("get_observation_body", &physics::Env::getObservationBody)
        .def("get_observation_joint", &physics::Env::getObservationJoint)
        .def("get_lines", &physics::Env::getLines)
        .def("get_center_xs", &physics::Env::getCenterXs)
        .def("get_center_ys", &physics::Env::getCenterYs)
        .def("get_energy_consumptions", &physics::Env::getEnergyConsumptions)
        .def("add_agent", &physics::Env::addAgent)
        .def("add_line", &physics::Env::addLine)
        .def("end_agent_assemble", &physics::Env::endAgentAssemble);
}
