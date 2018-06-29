///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include "basler_camera.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <memory>

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

i3ds::BaslerCamera::BaslerCamera(Context::Ptr context, NodeID node, Parameters param)
  : Camera(node),
    param_(param),
    publisher_(context, node)
{
  flash_enabled_ = false;
  flash_strength_ = 0.0;

  pattern_enabled_ = false;
  pattern_sequence_ = 0;
}

i3ds::BaslerCamera::~BaslerCamera()
{
  if (camera_)
    {
      camera_->Close();
      delete camera_;
    }
}

ShutterTime
i3ds::BaslerCamera::shutter() const
{
  // TODO: Confirm convertion.
  int exposureTimeRaw = 1; //camera_->ExposureTimeRaw.GetValue();
  float exposureTimeAbs = camera_->ExposureTimeAbs.GetValue();

  return (ShutterTime)((exposureTimeAbs * exposureTimeRaw) + 0.5);
}

SensorGain
i3ds::BaslerCamera::gain() const
{
  // TODO: Check need for convertion.
  return camera_->GainRaw.GetValue();
}

bool
i3ds::BaslerCamera::auto_exposure_enabled() const
{
  return camera_->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Continuous;
}

ShutterTime
i3ds::BaslerCamera::max_shutter() const
{
  // TODO: Check need for convertion.
  // return camera_->MaxShutterTime.GetValue();
  return 0;
}

SensorGain
i3ds::BaslerCamera::max_gain() const
{
  return camera_->GainRaw.GetMax();
}

bool
i3ds::BaslerCamera::region_enabled() const
{
  if (camera_->Width.GetValue() != camera_->Width.GetMax())
    {
      return false;
    }
  else if (camera_->Height.GetValue() != camera_->Height.GetMax())
    {
      return false;
    }
  else
    {
      return true;
    }
}

PlanarRegion
i3ds::BaslerCamera::region() const
{
  PlanarRegion region;

  region.size_x   = (T_UInt16) camera_->Width.GetValue();
  region.size_y   = (T_UInt16) camera_->Height.GetValue();
  region.offset_x = (T_UInt16) camera_->OffsetX.GetValue();
  region.offset_y = (T_UInt16) camera_->OffsetY.GetValue();

  return region;
}

void
i3ds::BaslerCamera::do_activate()
{
  BOOST_LOG_TRIVIAL(info) << "do_activate()";

  try
    {
      Pylon::CDeviceInfo info;

      info.SetUserDefinedName(param_.camera_name.c_str());
      info.SetDeviceClass(Pylon::CBaslerGigEInstantCamera::DeviceClass());

      camera_ = (Pylon::CBaslerGigEInstantCamera*) Pylon::CTlFactory::GetInstance().CreateFirstDevice(info);

      BOOST_LOG_TRIVIAL(info) << "Camera found:";

      //Print the model name of the camera.
      BOOST_LOG_TRIVIAL(info) << "Model name:    " << camera_->GetDeviceInfo().GetModelName();
      BOOST_LOG_TRIVIAL(info) << "Friendly name: " << camera_->GetDeviceInfo().GetFriendlyName();
      BOOST_LOG_TRIVIAL(info) << "Full name:     " << camera_->GetDeviceInfo().GetFullName();
      BOOST_LOG_TRIVIAL(info) << "SerialNumber:  " << camera_->GetDeviceInfo().GetSerialNumber();

      camera_->Open();

      //Switching format
      camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono12);

      // Set default sampling to reasonable value.
      if (param_.free_running)
        {
          camera_->AcquisitionFrameRateAbs.SetValue(10.0);
        }
    }
  catch (GenICam::GenericException &e)
    {
      throw i3ds::CommandError(error_other, "Error connecting to camera");;
    }
}

// \todo handle rate. What if not set?
void
i3ds::BaslerCamera::do_start()
{
  BOOST_LOG_TRIVIAL(info) << "do_start()";

  if (param_.free_running)
    {
      camera_->TriggerMode.SetValue(Basler_GigECamera::TriggerModeEnums::TriggerMode_Off);
      camera_->AcquisitionFrameRateEnable.SetValue(true);
      camera_->AcquisitionFrameRateAbs.SetValue(1.0 / period());
    }
  else
    {
      camera_->AcquisitionFrameRateEnable.SetValue(false);
      camera_->TriggerMode.SetValue(Basler_GigECamera::TriggerModeEnums::TriggerMode_On);
      camera_->TriggerSource.SetValue(Basler_GigECamera::TriggerSourceEnums::TriggerSource_Line1);
      camera_->TriggerSelector.SetValue(Basler_GigECamera::TriggerSelectorEnums::TriggerSelector_FrameStart);
    }

  sampler_ = std::thread(&BaslerCamera::SampleLoop, this);
}

void
i3ds::BaslerCamera::do_stop()
{
  BOOST_LOG_TRIVIAL(info) << "do_stop()";

  camera_->StopGrabbing();

  if (sampler_.joinable())
    {
      sampler_.join();
    }
}

void
i3ds::BaslerCamera::do_deactivate()
{
  camera_->Close();
  delete camera_;
  camera_ = nullptr;
}

bool
i3ds::BaslerCamera::is_sampling_supported(SampleCommand sample)
{
  BOOST_LOG_TRIVIAL(info) << "is_rate_supported() " << sample.period;

  // TODO: Add some real checking here, it might be different dependent on trigger.
  return 50000 <= sample.period && sample.period <= 10000000;
}

void
i3ds::BaslerCamera::handle_exposure(ExposureService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_exposure()";

  check_active();

  if (auto_exposure_enabled())
    {
      throw i3ds::CommandError(error_value, "In auto-exposure mode");
    }

  // TODO: Need convertion?
  camera_->ExposureTimeAbs.SetValue(1);
  camera_->ExposureTimeRaw.SetValue(command.request.shutter);
  camera_->GainRaw.SetValue(command.request.gain);
}

void
i3ds::BaslerCamera::handle_auto_exposure(AutoExposureService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_auto_exposure";

  check_active();

  if (command.request.enable)
    {
      camera_->ExposureAuto.SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous);
      //camera_->MaxShutterTime.SetValue(command.request.max_shutter);
    }
  else
    {
      camera_->ExposureAuto.SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off);
    }
}


void
i3ds::BaslerCamera::handle_region(RegionService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_region()";

  check_standby();

  // TODO: Reset to full resolution if enable is false.
  if (command.request.enable)
    {
      const PlanarRegion region = command.request.region;

      camera_->OffsetX.SetValue(region.offset_x);
      camera_->OffsetY.SetValue(region.offset_y);
      camera_->Width.SetValue(region.size_x);
      camera_->Height.SetValue(region.size_y);
    }
  else
    {
      camera_->OffsetX.SetValue(0);
      camera_->OffsetY.SetValue(0);
      camera_->Width.SetValue(camera_->Width.GetMax());
      camera_->Height.SetValue(camera_->Height.GetMax());
    }
}

void
i3ds::BaslerCamera::handle_flash(FlashService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_flash()";

  check_standby();

  flash_enabled_ = command.request.enable;

  if (command.request.enable)
    {
      flash_strength_ = command.request.strength;
    }
}

void
i3ds::BaslerCamera::handle_pattern(PatternService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_pattern()";

  check_standby();

  pattern_enabled_ = command.request.enable;

  if (command.request.enable)
    {
      pattern_sequence_ = command.request.sequence;
    }
}

bool
i3ds::BaslerCamera::send_sample(const byte* image, int width, int height)
{
  BOOST_LOG_TRIVIAL(info) << "BaslerCamera::send_sample()";

  Camera::FrameTopic::Data frame;

  Camera::FrameTopic::Codec::Initialize(frame);

  frame.descriptor.region.size_x = (T_UInt16) width;
  frame.descriptor.region.size_y = (T_UInt16) height;
  frame.descriptor.frame_mode = mode_mono;
  frame.descriptor.data_depth = param_.data_depth;
  frame.descriptor.pixel_size = 2;
  frame.descriptor.image_count = 1;

  const size_t size = image_size(frame.descriptor);

  frame.append_image(image, size);

  publisher_.Send<Camera::FrameTopic>(frame);

  return true;
}

void
i3ds::BaslerCamera::SampleLoop()
{
  BOOST_LOG_TRIVIAL(info) << "BaslerCamera::SampleLoop()";

  // The parameter MaxNumBuffer can be used to control the count of buffers
  // allocated for grabbing. The default value of this parameter is 10.
  camera_->MaxNumBuffer = 10;

  // create pylon image
  Pylon::CPylonImage pylonImage;

  // This smart pointer will receive the grab result data.
  Pylon::CGrabResultPtr ptrGrabResult;

  // Timeout of twice the period.
  int timeout_ms = period() / 500;

  // Start the grabbing of images.
  camera_->StartGrabbing();

  while (camera_->IsGrabbing())
    {
      // Wait for an image and then retrieve it.
      camera_->RetrieveResult(timeout_ms, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

      // Image grabbed successfully?
      if (ptrGrabResult->GrabSucceeded())
        {
          // Access the image data.
          const int width = ptrGrabResult->GetWidth();
          const int height = ptrGrabResult->GetHeight();

          const byte* pImageBuffer = (const byte*) ptrGrabResult->GetBuffer();

          send_sample(pImageBuffer, width, height);
        }
      else
        {
          BOOST_LOG_TRIVIAL(info) << "Error grabbing: "
                                  << ptrGrabResult->GetErrorCode() << " "
                                  << ptrGrabResult->GetErrorDescription();
        }
    }
}
