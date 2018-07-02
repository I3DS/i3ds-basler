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

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <cmath>

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

  camera_ = nullptr;

  Pylon::PylonInitialize();
}

i3ds::BaslerCamera::~BaslerCamera()
{
  if (camera_)
    {
      camera_->Close();
      delete camera_;
    }
}

SensorGain
i3ds::BaslerCamera::raw_to_gain(int64_t raw) const
{
  // TODO: Camera specific for ACE acA2040-25gmNIR.
  return 20.0 * log10((1.0 / 32.0) * raw);
}

int64_t
i3ds::BaslerCamera::gain_to_raw(SensorGain gain) const
{
  // TODO: Camera specific for ACE acA2040-25gmNIR.
  return (int64_t) 32.0 * pow(10, gain / 20.0);
}


ShutterTime
i3ds::BaslerCamera::shutter() const
{
  return (ShutterTime) camera_->ExposureTimeAbs.GetValue();
}

SensorGain
i3ds::BaslerCamera::gain() const
{
  return raw_to_gain(camera_->GainRaw.GetValue());
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
  // TODO: Does this use another conversion?
  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetValue());
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

      BOOST_LOG_TRIVIAL(info) << "Searching for camera named: " << param_.camera_name;

      // TODO: Fix this convertion.
      camera_ = new Pylon::CBaslerGigEInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice(info));

      BOOST_LOG_TRIVIAL(info) << "Camera found!";

      // Print the model name of the camera.
      BOOST_LOG_TRIVIAL(info) << "Model name:    " << camera_->GetDeviceInfo().GetModelName();
      BOOST_LOG_TRIVIAL(info) << "Friendly name: " << camera_->GetDeviceInfo().GetFriendlyName();
      BOOST_LOG_TRIVIAL(info) << "Full name:     " << camera_->GetDeviceInfo().GetFullName();
      BOOST_LOG_TRIVIAL(info) << "SerialNumber:  " << camera_->GetDeviceInfo().GetSerialNumber();

      // Open camera.
      camera_->Open();

      // Set streaming options.
      camera_->GevSCPSPacketSize.SetValue(param_.packet_size);
      camera_->GevSCPD.SetValue(param_.packet_delay);

      // Switching format
      camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono12);

      // Set default sampling to reasonable value.
      if (param_.free_running)
        {
          camera_->AcquisitionFrameRateAbs.SetValue(10.0);
        }
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();

      throw i3ds::CommandError(error_other, "Error connecting to camera");;
    }
}

void
i3ds::BaslerCamera::do_start()
{
  using namespace Basler_GigECamera;

  BOOST_LOG_TRIVIAL(info) << "do_start()";

  if (param_.free_running)
    {
      double fps = 1.0e6 / period();
      double fps_min = camera_->AcquisitionFrameRateAbs.GetMin();
      double fps_max = camera_->AcquisitionFrameRateAbs.GetMax();

      if (fps < fps_min) fps = fps_min;
      if (fps < fps_max) fps = fps_max;

      BOOST_LOG_TRIVIAL(info) << "Free running at " << fps << " FPS";

      camera_->TriggerMode.SetValue(TriggerMode_Off);

      camera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
      camera_->AcquisitionFrameRateEnable.SetValue(true);
      camera_->AcquisitionFrameRateAbs.SetValue(1.0e6 / period());
    }
  else
    {
      camera_->AcquisitionFrameRateEnable.SetValue(false);

      camera_->TriggerSelector.SetValue(TriggerSelector_FrameStart);
      camera_->TriggerMode.SetValue(TriggerMode_On);
      camera_->TriggerSource.SetValue(TriggerSource_Line1);
      camera_->ExposureMode.SetValue(ExposureMode_Timed);
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

  const double fps = 1.0e6 / sample.period;
  const double fps_min = camera_->AcquisitionFrameRateAbs.GetMin();
  const double fps_max = camera_->AcquisitionFrameRateAbs.GetMax();

  BOOST_LOG_TRIVIAL(info) <<  "fps: " << fps
			  << " min: " << fps_min
			  << " max: " << fps_max;

  return fps_min <= fps && fps <= fps_max;
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

  int64_t shutter = command.request.shutter;
  int64_t shutter_max = camera_->ExposureTimeRaw.GetMax();
  int64_t shutter_min = camera_->ExposureTimeRaw.GetMin();

  if (shutter > (int64_t) period())
    {
      throw i3ds::CommandError(error_value, "Shutter longer than period!");
    }

  if (shutter > shutter_max)
    {
      throw i3ds::CommandError(error_value, "Shutter longer than " + std::to_string(shutter_max));
    }

  if (shutter < shutter_min)
    {
      throw i3ds::CommandError(error_value, "Shutter shorter than " + std::to_string(shutter_min));
    }

  int64_t raw = gain_to_raw(command.request.gain);
  int64_t raw_max = camera_->GainRaw.GetMax();
  int64_t raw_min = camera_->GainRaw.GetMin();

  if (raw > raw_max)
    {
      throw i3ds::CommandError(error_value, "Gain higher than " + std::to_string(raw_to_gain(raw_max)));
    }

  if (raw < raw_min)
    {
      throw i3ds::CommandError(error_value, "Gain lower than " + std::to_string(raw_to_gain(raw_min)));
    }

  try
    {
      camera_->ExposureTimeRaw.SetValue(shutter);
      camera_->GainRaw.SetValue(raw);
    }
  catch (GenICam::GenericException& e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();
    }
}

void
i3ds::BaslerCamera::handle_auto_exposure(AutoExposureService::Data& command)
{
  using namespace Basler_GigECamera;

  BOOST_LOG_TRIVIAL(info) << "handle_auto_exposure";

  check_active();

  if (command.request.enable)
    {
      double min_gain = camera_->AutoGainRawLowerLimit.GetMin();
      double max_gain = gain_to_raw(command.request.max_gain);

      camera_->AutoGainRawLowerLimit.SetValue(min_gain);
      camera_->AutoGainRawUpperLimit.SetValue(max_gain);

      camera_->AutoFunctionProfile.SetValue(AutoFunctionProfile_GainMinimum);
      camera_->GainAuto.SetValue(GainAuto_Continuous);
      camera_->ExposureAuto.SetValue(ExposureAuto_Continuous);
    }
  else
    {
      camera_->GainAuto.SetValue(GainAuto_Off);
      camera_->ExposureAuto.SetValue(ExposureAuto_Off);
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
  BOOST_LOG_TRIVIAL(info) << "BaslerCamera::send_sample() (" << width << "x" << height << ")" ;

  Camera::FrameTopic::Data frame;

  Camera::FrameTopic::Codec::Initialize(frame);

  frame.descriptor.attributes.timestamp = 1;
  frame.descriptor.attributes.validity = sample_valid;

  frame.descriptor.region.offset_x = 0;
  frame.descriptor.region.offset_y = 0;
  frame.descriptor.region.size_x = (T_UInt16) width;
  frame.descriptor.region.size_y = (T_UInt16) height;

  frame.descriptor.frame_mode = mode_mono;
  frame.descriptor.data_depth = 12;
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
      try
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
      catch (TimeoutException& e)
	{
	  BOOST_LOG_TRIVIAL(warning) << "TIMEOUT!";
	}
      catch (GenICam::GenericException& e)
	{
	  BOOST_LOG_TRIVIAL(warning) << e.what();
	  break;
	}
    }
}
