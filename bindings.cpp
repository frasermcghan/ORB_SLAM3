#include "System.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(orbslam3, m) {
  py::class_<ORB_SLAM3::System> system(m, "System");

  system.def(py::init<const std::string &, const std::string &,
                      ORB_SLAM3::System::eSensor, const bool>());

  system.def("TrackRGBD", &ORB_SLAM3::System::TrackRGBD_Eigen);
  system.def("Reset", &ORB_SLAM3::System::Reset);
  system.def("GetTrackingState", &ORB_SLAM3::System::GetTrackingState);
  system.def("GetViewerFrame", &ORB_SLAM3::System::GetViewerFrame);

  py::enum_<ORB_SLAM3::System::eSensor>(system, "SensorType")
      .value("MONOCULAR", ORB_SLAM3::System::eSensor::MONOCULAR)
      .value("STEREO", ORB_SLAM3::System::eSensor::STEREO)
      .value("RGBD", ORB_SLAM3::System::eSensor::RGBD)
      .value("IMU_MONOCULAR", ORB_SLAM3::System::eSensor::IMU_MONOCULAR)
      .value("IMU_STEREO", ORB_SLAM3::System::eSensor::IMU_STEREO)
      .value("IMU_RGBD", ORB_SLAM3::System::eSensor::IMU_RGBD)
      .export_values();

  py::class_<cv::Mat>(m, "Image", pybind11::buffer_protocol())
      .def_buffer([](cv::Mat &im) -> pybind11::buffer_info {
        return pybind11::buffer_info(
            im.data, sizeof(unsigned char),
            pybind11::format_descriptor<unsigned char>::format(), 3,
            {im.rows, im.cols, im.channels()},
            {sizeof(unsigned char) * im.channels() * im.cols,
             sizeof(unsigned char) * im.channels(), sizeof(unsigned char)});
      });
}
