
///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////


#ifndef __PYLON_WRAPPER_HPP
#define __PYLON_WRAPPER_HPP

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>

#include <thread>

// Does sampling operation, returns true if more samples are requested.
typedef std::function<bool(unsigned char* image, int width, int height)> Operation;

class PylonWrapper
{
public:

  PylonWrapper(std::string camera_name, Operation operation);

  void stopSampling();
  void startSamplingLoop();
  void startSampling();
  void openForParameterManipulation();
  void closeForParameterManipulation();
  void do_activate();
  void do_start();
  void do_stop();
  void do_deactivate();

  void connect();

  void setGain(int64_t value);
  int64_t getGain();

  int64_t getShutterTime();
  void setShutterTime(int64_t value);

  void setRegionEnabled(bool regionEnabled);
  bool getRegionEnabled();

  void setTriggerInterval(int64_t);
  bool checkTriggerInterval(int64_t);

  int64_t getMaxShutterTime();
  void setMaxShutterTime(int64_t);

  void setRegion(PlanarRegion region);
  PlanarRegion getRegion();

  bool getAutoExposureEnabled();
  void setAutoExposureEnabled(bool enabled);

private:

  std::unique_ptr<Pylon::CBaslerGigEInstantCamera> camera_;

  // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
  // is initialized during the lifetime of this object.
  Pylon::PylonAutoInitTerm autoInitTerm;

  std::thread threadSamplingLoop;

  std::string cameraName_;

  // Sample operation.
  std::string connection_id_;

  Operation operation_;
};

#endif
