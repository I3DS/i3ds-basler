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

#include <i3ds/gige_camera_sensor.hpp>

#include <thread>
#include <memory>

#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>

namespace i3ds
{

class BaslerCamera : public GigECamera
{
public:

  BaslerCamera(Context::Ptr context, i3ds_asn1::NodeID node, Parameters param);

  virtual ~BaslerCamera();

protected:

  // Camera control
  virtual void Open() override;
  virtual void Close() override;
  virtual void Start() override;
  virtual void Stop() override;

  // Set internal trigger to the given period.
  virtual bool setInternalTrigger(int64_t period_us) override;

  // Sensor width and height
  virtual int64_t getSensorWidth() const override;
  virtual int64_t getSensorHeight() const override;

  // Region of interest
  virtual bool isRegionSupported() const override;

  virtual int64_t getRegionWidth() const override;
  virtual int64_t getRegionHeight() const override;
  virtual int64_t getRegionOffsetX() const override;
  virtual int64_t getRegionOffsetY() const override;

  virtual void setRegionWidth(int64_t width) override;
  virtual void setRegionHeight(int64_t height) override;
  virtual void setRegionOffsetX(int64_t offset_x) override;
  virtual void setRegionOffsetY(int64_t offset_y) override;

  // Shutter time in microseconds
  virtual int64_t getShutter() const override;
  virtual int64_t getMaxShutter() const override;
  virtual int64_t getMinShutter() const override;
  virtual void setShutter(int64_t shutter_us) override;

  virtual bool isAutoShutterSupported() const override;

  virtual bool getAutoShutterEnabled() const override;
  virtual void setAutoShutterEnabled(bool enable) override;

  virtual int64_t getAutoShutterLimit() const override;
  virtual int64_t getMaxAutoShutterLimit() const override;
  virtual int64_t getMinAutoShutterLimit() const override;
  virtual void setAutoShutterLimit(int64_t shutter_limit) override;

  // Gain in decibel
  virtual double getGain() const override;
  virtual double getMaxGain() const override;
  virtual double getMinGain() const override;
  virtual void setGain(double gain) override;

  virtual bool isAutoGainSupported() const override;

  virtual bool getAutoGainEnabled() const override;
  virtual void setAutoGainEnabled(bool enable) override;

  virtual double getAutoGainLimit() const override;
  virtual double getMaxAutoGainLimit() const override;
  virtual double getMinAutoGainLimit() const override;
  virtual void setAutoGainLimit(double gain_limit) override;

private:

  double raw_to_gain(int64_t raw) const;
  int64_t gain_to_raw(double gain) const;

  void set_error_state(const std::string &error_message, const bool dont_throw );

  void SampleLoop();

  int retrive_errors_ ;
  int timeout_counter_ ;
  int grab_errors_;

  std::thread sampler_;

  mutable Pylon::CBaslerGigEInstantCamera* camera_;
};

} // namespace i3ds

#endif
