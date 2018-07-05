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

#include <i3ds/topic.hpp>
#include <i3ds/publisher.hpp>
#include <i3ds/camera_sensor.hpp>
#include <i3ds/trigger_client.hpp>

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
    int packet_size;
    int packet_delay;
    int trigger_source;
    int camera_output;
    int camera_offset;
    int flash_output;
    int flash_offset;
    int pattern_output;
    int pattern_offset;
  };

  BaslerCamera(Context::Ptr context, NodeID id, Parameters param, TriggerClient::Ptr trigger = nullptr);
  virtual ~BaslerCamera();

  // Getters.
  virtual ShutterTime shutter() const;
  virtual SensorGain gain() const;

  virtual bool auto_gain_enabled() const;

  virtual void handle_auto_exposure_helper(const int64_t max_shutter_time, const int64_t max_gain_parameter) const;

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

  SensorGain raw_to_gain(int64_t raw) const;
  int64_t gain_to_raw(SensorGain gain) const;

  void SampleLoop();
  bool send_sample(const byte* image, int width, int height);

  void set_trigger(TriggerOutput channel, TriggerOffset offset);
  void clear_trigger(TriggerOutput channel);

  bool flash_enabled_;
  FlashStrength flash_strength_;

  bool pattern_enabled_;
  PatternSequence pattern_sequence_;

  Publisher publisher_;
  TriggerClient::Ptr trigger_;
  TriggerMask trigger_mask_;

  std::thread sampler_;

  mutable Pylon::CBaslerGigEInstantCamera* camera_;
};

} // namespace i3ds

#endif
