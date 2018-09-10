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

using namespace Basler_GigECamera;

i3ds::BaslerCamera::BaslerCamera(Context::Ptr context, NodeID node, Parameters param)
  : GigECamera(context, node, param)
{
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

void
i3ds::BaslerCamera::Open()
{
  BOOST_LOG_TRIVIAL(info) << "Open()";

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

      // Set device name.
      set_device_name(camera_->GetDeviceInfo().GetFriendlyName().c_str());
      
      // Open camera.
      camera_->Open();

      BOOST_LOG_TRIVIAL(info) << "param_.packet_size: " << param_.packet_size;
      BOOST_LOG_TRIVIAL(info) << "packet_delay: " << param_.packet_delay;

      // Set streaming options.
      camera_->GevSCPSPacketSize.SetValue(param_.packet_size);
      camera_->GevSCPD.SetValue(param_.packet_delay);

      // Set exposure time base to 10 us if enabled.
      if (camera_->ExposureTimeBaseAbsEnable.GetValue())
	{
	  // TODO: Make this a parameter?
	  camera_->ExposureTimeBaseAbs.SetValue(10);
	}

      // Set pixel format depending on data depth.
      switch (param_.data_depth)
	{
	case 12:
	  BOOST_LOG_TRIVIAL(info) << "Pixel format: Mono12";
	  camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono12);
	  break;

	case 8:
	  BOOST_LOG_TRIVIAL(info) << "Pixel format: Mono8";
	  camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono8);
	  break;

	default:
	  BOOST_LOG_TRIVIAL(error) << "Unsupported data depth: " << param_.data_depth;
	}
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();

      throw i3ds::CommandError(error_other, "Error connecting to camera");;
    }
}

void
i3ds::BaslerCamera::Close()
{
  BOOST_LOG_TRIVIAL(info) << "Close()";

  camera_->Close();
  delete camera_;

  camera_ = nullptr;
}

void
i3ds::BaslerCamera::Start()
{
  BOOST_LOG_TRIVIAL(info) << "Start()";

  try
    {
      if (param_.external_trigger)
	{
	  BOOST_LOG_TRIVIAL(info) << "Setting external trigger values";
	  
	  camera_->AcquisitionFrameRateEnable.SetValue(false);

	  camera_->TriggerSelector.SetValue(TriggerSelector_FrameStart);
	  camera_->TriggerMode.SetValue(TriggerMode_On);
	  camera_->TriggerSource.SetValue(TriggerSource_Line1);
	  camera_->ExposureMode.SetValue(ExposureMode_Timed);
	}
      else
	{
	  BOOST_LOG_TRIVIAL(info) << "Setting internal trigger values";
	  
	  camera_->TriggerMode.SetValue(TriggerMode_Off);
	  camera_->AcquisitionFrameRateEnable.SetValue(true);
	  camera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
	  camera_->AcquisitionFrameRateAbs.SetValue(1.0e6 / period());
	}
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();

      throw i3ds::CommandError(error_other, "Error starting camera");;
    }

  sampler_ = std::thread(&i3ds::BaslerCamera::SampleLoop, this);
}

void
i3ds::BaslerCamera::Stop()
{
  BOOST_LOG_TRIVIAL(info) << "Stop()";

  try
    {
      camera_->StopGrabbing();
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();
    }

  if (sampler_.joinable())
    {
      sampler_.join();
    }
}

bool
i3ds::BaslerCamera::setInternalTrigger(int64_t period_us)
{
  BOOST_LOG_TRIVIAL(info) << "setInternalTrigger()";

  const float rate = 1.e6 / period_us;

  camera_->AcquisitionFrameRateEnable.SetValue(true);

  try
    {
      camera_->AcquisitionFrameRateAbs.SetValue(rate);
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(error) << "Frame rate is out of range: " << rate;

      return false;
    }

  return true;
}

int64_t
i3ds::BaslerCamera::getSensorWidth() const
{
  BOOST_LOG_TRIVIAL(info) << "getSensorWidth()";

  return camera_->Width.GetMax();
}

int64_t
i3ds::BaslerCamera::getSensorHeight() const
{
  BOOST_LOG_TRIVIAL(info) << "getSensorHeight()";

  return camera_->Height.GetMax();
}

bool
i3ds::BaslerCamera::isRegionSupported() const
{
  BOOST_LOG_TRIVIAL(info) << "isRegionSupported()";

  return true;
}

int64_t
i3ds::BaslerCamera::getRegionWidth() const
{
  BOOST_LOG_TRIVIAL(info) << "getRegionWidth()";

  return camera_->Width.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionHeight() const
{
  BOOST_LOG_TRIVIAL(info) << "getRegionHeight()";
  
  return camera_->Height.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionOffsetX() const
{
  BOOST_LOG_TRIVIAL(info) << "getRegionOffsetX()";
  
  return camera_->OffsetX.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionOffsetY() const
{
  BOOST_LOG_TRIVIAL(info) << "getRegionOffsetY()";
  
  return camera_->OffsetY.GetValue();
}

void
i3ds::BaslerCamera::setRegionWidth(int64_t width)
{
  BOOST_LOG_TRIVIAL(info) << "setRegionWidth()";
  
  camera_->Width.SetValue(width);
}

void
i3ds::BaslerCamera::setRegionHeight(int64_t height)
{
  BOOST_LOG_TRIVIAL(info) << "setRegionHeight()";
  
  camera_->Height.SetValue(height);
}

void
i3ds::BaslerCamera::setRegionOffsetX(int64_t offset_x)
{
  BOOST_LOG_TRIVIAL(info) << "setRegionOffsetX()";
  
  camera_->OffsetX.SetValue(offset_x);
}

void
i3ds::BaslerCamera::setRegionOffsetY(int64_t offset_y)
{
  BOOST_LOG_TRIVIAL(info) << "setRegionOffsetY()";
  
  camera_->OffsetY.SetValue(offset_y);
}

int64_t
i3ds::BaslerCamera::getShutter() const
{
  BOOST_LOG_TRIVIAL(info) << "getShutter()";
  
  return camera_->ExposureTimeAbs.GetValue();
}

int64_t
i3ds::BaslerCamera::getMaxShutter() const
{
  BOOST_LOG_TRIVIAL(info) << "getMaxShutter()";
  
  return camera_->ExposureTimeAbs.GetMax();
}

int64_t
i3ds::BaslerCamera::getMinShutter() const
{
  BOOST_LOG_TRIVIAL(info) << "getMinShutter()";
  
  return camera_->ExposureTimeAbs.GetMin();
}

void
i3ds::BaslerCamera::setShutter(int64_t shutter_us)
{
  BOOST_LOG_TRIVIAL(info) << "setShutter()";
  
  camera_->ExposureTimeAbs.SetValue(shutter_us);
}

bool
i3ds::BaslerCamera::isAutoShutterSupported() const
{
  BOOST_LOG_TRIVIAL(info) << "isAutoShutterSupported()";
  
  return true;
}

bool
i3ds::BaslerCamera::getAutoShutterEnabled() const
{
  BOOST_LOG_TRIVIAL(info) << "getAutoShutterEnabled()";
  
  return camera_->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Continuous;
}

void
i3ds::BaslerCamera::setAutoShutterEnabled(bool enable)
{
  BOOST_LOG_TRIVIAL(info) << "setAutoShutterEnabled()";
  
  if (enable)
    {
      // TODO: Handle that some cameras don't support profiles.
      //camera_->AutoFunctionProfile.SetValue(AutoFunctionProfile_GainMinimum);
      camera_->ExposureAuto.SetValue(ExposureAuto_Continuous);
    }
  else
    {
      camera_->ExposureAuto.SetValue(ExposureAuto_Off);
    }
}

int64_t
i3ds::BaslerCamera::getAutoShutterLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getAutoShutterLimit()";
  
  return camera_->AutoExposureTimeAbsUpperLimit.GetValue();
}

int64_t
i3ds::BaslerCamera::getMaxAutoShutterLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getMaxAutoShutterLimit()";
  
  return camera_->AutoExposureTimeAbsUpperLimit.GetMax();
}

int64_t
i3ds::BaslerCamera::getMinAutoShutterLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getMinAutoShutterLimit()";
  
  return camera_->AutoExposureTimeAbsUpperLimit.GetMin();
}

void
i3ds::BaslerCamera::setAutoShutterLimit(int64_t shutter_limit)
{
  BOOST_LOG_TRIVIAL(info) << "setAutoShutterLimit()";
  
  const double lower = camera_->AutoExposureTimeAbsLowerLimit.GetMin();
  const double upper = shutter_limit;

  camera_->AutoExposureTimeAbsLowerLimit.SetValue(lower);
  camera_->AutoExposureTimeAbsUpperLimit.SetValue(upper);
}

double
i3ds::BaslerCamera::getGain() const
{
  BOOST_LOG_TRIVIAL(info) << "getGain()";
  
  return raw_to_gain(camera_->GainRaw.GetValue());
}

double
i3ds::BaslerCamera::getMaxGain() const
{
  BOOST_LOG_TRIVIAL(info) << "getMaxGain()";
  
  return raw_to_gain(camera_->GainRaw.GetMax());
}

double
i3ds::BaslerCamera::getMinGain() const
{
  BOOST_LOG_TRIVIAL(info) << "getMinGain()";
  
  return raw_to_gain(camera_->GainRaw.GetMin());
}

void
i3ds::BaslerCamera::setGain(double gain)
{
  BOOST_LOG_TRIVIAL(info) << "setGain()";
  
  camera_->GainRaw.SetValue(gain_to_raw(gain));
}

bool
i3ds::BaslerCamera::isAutoGainSupported() const
{
  BOOST_LOG_TRIVIAL(info) << "isAutoGainSupported()";

  // TODO: Handle that some cameras don't support auto gain and shutter at same time.
  return false;
}

bool
i3ds::BaslerCamera::getAutoGainEnabled() const
{
  BOOST_LOG_TRIVIAL(info) << "getAutoGainEnabled()";
  
  return camera_->GainAuto.GetValue() == GainAuto_Continuous;
}

void
i3ds::BaslerCamera::setAutoGainEnabled(bool enable)
{
  BOOST_LOG_TRIVIAL(info) << "setAutoGainEnabled()";
  
  if (enable)
    {
      // TODO: Handle that some cameras don't support profiles.
      //camera_->AutoFunctionProfile.SetValue(AutoFunctionProfile_GainMinimum);
      camera_->GainAuto.SetValue(GainAuto_Continuous);
    }
  else
    {
      camera_->GainAuto.SetValue(GainAuto_Off);
    }
}

double
i3ds::BaslerCamera::getAutoGainLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getAutoGainLimit()";
  
  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetValue());
}

double
i3ds::BaslerCamera::getMaxAutoGainLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getMaxAutoGainLimit()";
  
  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetMax());
}

double
i3ds::BaslerCamera::getMinAutoGainLimit() const
{
  BOOST_LOG_TRIVIAL(info) << "getMinAutoGainLimit()";
  
  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetMin());
}

void
i3ds::BaslerCamera::setAutoGainLimit(double gain_limit)
{
  BOOST_LOG_TRIVIAL(info) << "setAutoGainLimit()";
  
  const int64_t lower = camera_->AutoGainRawLowerLimit.GetMin();
  const int64_t upper = gain_to_raw(gain_limit);

  camera_->AutoGainRawLowerLimit.SetValue(lower);
  camera_->AutoGainRawUpperLimit.SetValue(upper);
}

double
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

void
i3ds::BaslerCamera::SampleLoop()
{
  BOOST_LOG_TRIVIAL(info) << "SampleLoop()";

  // The parameter MaxNumBuffer can be used to control the count of buffers
  // allocated for grabbing. The default value of this parameter is 10.
  camera_->MaxNumBuffer = 10;

  // create pylon image
  Pylon::CPylonImage pylonImage;

  // This smart pointer will receive the grab result data.
  Pylon::CGrabResultPtr ptrGrabResult;

  // Timeout 4 times a second. The only thing we are waiting for is actually a stopline.
  // Else we always are waiting for a image.
  const int timeout_ms = 250;

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
              BOOST_LOG_TRIVIAL(warning) << "Error grabbing: "
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
