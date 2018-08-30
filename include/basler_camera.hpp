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

  BaslerCamera(Context::Ptr context, NodeID node, Parameters param);

  virtual ~BaslerCamera();

protected:

  // Camera control
  virtual void Open();
  virtual void Close();
  virtual void Start();
  virtual void Stop();

  // Set internal trigger to the given period.
  virtual bool setInternalTrigger(int64_t period_us);

  // Sensor width and height
  virtual int64_t getSensorWidth() const;
  virtual int64_t getSensorHeight() const;

  // Region of interest
  virtual bool isRegionSupported() const;

  virtual int64_t getRegionWidth() const;
  virtual int64_t getRegionHeight() const;
  virtual int64_t getRegionOffsetX() const;
  virtual int64_t getRegionOffsetY() const;

  virtual void setRegionWidth(int64_t width);
  virtual void setRegionHeight(int64_t height);
  virtual void setRegionOffsetX(int64_t offset_x);
  virtual void setRegionOffsetY(int64_t offset_y);

  // Shutter time in microseconds
  virtual int64_t getShutter() const;
  virtual int64_t getMaxShutter() const;
  virtual int64_t getMinShutter() const;
  virtual void setShutter(int64_t shutter_us);

  virtual bool isAutoShutterSupported() const;

  virtual bool getAutoShutterEnabled() const;
  virtual void setAutoShutterEnabled(bool enable);

  virtual int64_t getAutoShutterLimit() const;
  virtual int64_t getMaxAutoShutterLimit() const;
  virtual int64_t getMinAutoShutterLimit() const;
  virtual void setAutoShutterLimit(int64_t shutter_limit);

  // Gain in decibel
  virtual double getGain() const;
  virtual double getMaxGain() const;
  virtual double getMinGain() const;
  virtual void setGain(double gain);

  virtual bool isAutoGainSupported() const;

  virtual bool getAutoGainEnabled() const;
  virtual void setAutoGainEnabled(bool enable);

  virtual double getAutoGainLimit() const;
  virtual double getMaxAutoGainLimit() const;
  virtual double getMinAutoGainLimit() const;
  virtual void setAutoGainLimit(double gain_limit);

private:

  double raw_to_gain(int64_t raw) const;
  int64_t gain_to_raw(double gain) const;

  void SampleLoop();

  std::thread sampler_;

  mutable Pylon::CBaslerGigEInstantCamera* camera_;
};

} // namespace i3ds

#endif
