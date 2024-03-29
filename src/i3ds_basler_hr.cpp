///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <csignal>
#include <iostream>
#include <unistd.h>
#include <string>
#include <vector>
#include <memory>

#include <boost/program_options.hpp>

#include <i3ds/exception.hpp>
#include <i3ds/configurator.hpp>
#include "i3ds/communication.hpp"
#include "basler_camera.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>

namespace po = boost::program_options;
namespace logging = boost::log;

volatile bool running;

void signal_handler(int signum)
{
  BOOST_LOG_TRIVIAL(info) << "do_deactivate()";
  running = false;
}

struct counter { int count = 0; };
void validate(boost::any& v, std::vector<std::string> const& xs, counter*, long)
{
  if (v.empty()) {
    counter num;
    num.count = 1;
    v = num;
  } else {
    ++boost::any_cast<counter&>(v).count;
  }
}

int main(int argc, char** argv)
{
  unsigned int node_id;

  bool rgb;
  bool yuv;

  bool enable_trigger_output;
  double gamma;
  int black_level;

  i3ds::GigECamera::Parameters param;
  i3ds::Configurator configurator;

  po::options_description desc("Allowed camera control options");
  configurator.add_common_options(desc);

  desc.add_options()
  ("node,n", po::value<unsigned int>(&node_id)->default_value(10), "Node ID of camera")

  ("camera-name,c", po::value<std::string>(&param.camera_name), "Connect via (UserDefinedName) of Camera")
  ("package-size", po::value<int>(&param.packet_size)->default_value(8192), "Transport-layer buffersize (MTU).")
  ("package-delay", po::value<int>(&param.packet_delay)->default_value(20), "Inter-package delay parameter of camera.")
  ("data-depth", po::value<int>(&param.data_depth)->default_value(12), "Depth of image pixels (bits).")

  ("trigger", po::value<bool>(&param.external_trigger)->default_value(true), "Enable external trigger.")
  ("trigger-node", po::value<i3ds_asn1::NodeID>(&param.trigger_node)->default_value(20), "Node ID of trigger service.")
  ("trigger-source", po::value<int>(&param.trigger_source)->default_value(1), "Trigger generator for camera.")
  ("trigger-camera-output", po::value<int>(&param.camera_output)->default_value(2), "Trigger output for camera.")
  ("trigger-camera-offset", po::value<int>(&param.camera_offset)->default_value(5000), "Trigger offset for camera (us).")

  ("trigger-out", po::value<bool>(&enable_trigger_output)->default_value(false), "Enables opto-GPIO output on exposure")

  ("flash", po::value<bool>(&param.support_flash)->default_value(false), "Support wide-angle flash.")
  ("flash-node", po::value<i3ds_asn1::NodeID>(&param.flash_node)->default_value(21), "Node ID of flash service.")
  ("trigger-flash-output", po::value<int>(&param.flash_output)->default_value(8), "Trigger output for flash.")
  ("trigger-flash-offset", po::value<int>(&param.flash_offset)->default_value(4200), "Trigger offset for flash (us).")

  ("pattern", po::value<bool>(&param.support_pattern)->default_value(false), "Support pattern illumination.")
  ("trigger-pattern-output", po::value<int>(&param.pattern_output)->default_value(6), "Trigger output for pattern.")
  ("trigger-pattern-offset", po::value<int>(&param.pattern_offset)->default_value(0), "Trigger offset for pattern (us).")
  ("rgb", po::value<bool>(&rgb)->default_value(false), "Capture RGB images.")
  ("yuv", po::value<bool>(&yuv)->default_value(false), "Capture YUV422 images.")
  ("black-level", po::value<int>(&black_level), "Adjust the overall brightness by adjusting ech pixel [0..255]")
  ("disable-gamma", "Disable gamma correction value.")
  ("gamma", po::value<double>(&gamma), "Set Gamma correction value [0..4.0] (will set black level to 0 unless overidden)")
  ("print,p", "Print the camera configuration")
  ;

  po::variables_map vm = configurator.parse_common_options(desc, argc, argv);

  if (param.data_depth < 8 || param.data_depth > 12)
      throw i3ds::CommandError(i3ds_asn1::ResultCode_error_unsupported, "data_depth out of range.");
  param.pixel_size = param.data_depth / 8 + (param.data_depth % 8 > 0);

  BOOST_LOG_TRIVIAL(info) << "Node ID:     " << node_id;
  BOOST_LOG_TRIVIAL(info) << "Camera name: " << param.camera_name;
  BOOST_LOG_TRIVIAL(info) << "Camera type: Basler HR";
  BOOST_LOG_TRIVIAL(trace) << "Data-depth: " << param.data_depth;
  BOOST_LOG_TRIVIAL(trace) << "pixel-size: " << param.pixel_size;

  param.image_count = 1;

  if (rgb)
    {
      param.frame_mode = i3ds_asn1::Frame_mode_t_mode_rgb;
      param.pixel_size *= 3;
    }
  else if (yuv)
    {
      param.frame_mode = i3ds_asn1::Frame_mode_t_mode_uyvy;
      param.pixel_size *= 2;
    }
  else
    {
      param.frame_mode = i3ds_asn1::Frame_mode_t_mode_mono;
    }

  i3ds::Context::Ptr context = i3ds::Context::Create();;

  i3ds::Server server(context);

  i3ds::BaslerCamera camera(context, node_id, param, enable_trigger_output);

  camera.Attach(server);

  running = true;
  signal(SIGINT, signal_handler);

  server.Start();

  // Handle black & gamma, this can only be done after activate, so this
  // won't happen for a while yet.
  if (vm.count("black-level"))
      camera.overrideBlack(black_level);

  if (vm.count("disable-gamma")) {
      camera.overrideGamma(false);
  }
  else if (vm.count("gamma")) {
      camera.overrideGamma(true, gamma);
  }

  while (running)
    {
      sleep(1);
    }

  server.Stop();

  return 0;
}
