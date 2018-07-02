///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////


#ifndef __BASLER_CAMERA_HPP
#define __BASLER_CAMERA_HPP

#include "i3ds/topic.hpp"
#include "i3ds/publisher.hpp"
#include "i3ds/camera_sensor.hpp"

#include <thread>

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>

namespace i3ds
{

class BaslerCamera : public Camera
{
public:

  struct Parameters
  {
    std::string camera_name;
    bool free_running;
    int data_depth;
  };

  BaslerCamera(Context::Ptr context, NodeID id, Parameters param);
  virtual ~BaslerCamera();

  // Getters.
  virtual ShutterTime shutter() const;
  virtual SensorGain gain() const;

  virtual bool auto_exposure_enabled() const;
  virtual ShutterTime max_shutter() const;
  virtual SensorGain max_gain() const;

  virtual bool region_enabled() const;
  virtual PlanarRegion region() const;

  virtual bool flash_enabled() const {return flash_enabled_;}
  virtual FlashStrength flash_strength() const {return flash_strength_;}

  virtual bool pattern_enabled() const {return pattern_enabled_;}
  virtual PatternSequence pattern_sequence() const {return pattern_sequence_;}

  virtual bool is_sampling_supported(SampleCommand sample);

protected:

  // Actions.
  virtual void do_activate();
  virtual void do_start();
  virtual void do_stop();
  virtual void do_deactivate();

  // Handlers.
  virtual void handle_exposure(ExposureService::Data& command);
  virtual void handle_region(RegionService::Data& command);
  virtual void handle_auto_exposure(AutoExposureService::Data& command);
  virtual void handle_flash(FlashService::Data& command);
  virtual void handle_pattern(PatternService::Data& command);

private:

  const Parameters param_;

  void SampleLoop();
  bool send_sample(const byte* image, int width, int height);

  void updateRegion();

  bool flash_enabled_;
  FlashStrength flash_strength_;

  bool pattern_enabled_;
  PatternSequence pattern_sequence_;

  Publisher publisher_;

  Pylon::CBaslerGigEInstantCamera* camera_;
  std::thread sampler_;
};

} // namespace i3ds

#endif