///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2018 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
///////////////////////////////////////////////////////////////////////////////

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>

#include <pylon/gige/BaslerGigEInstantCamera.h>

#include <thread>
#include <chrono>

#include "pylon_wrapper.hpp"

#define BOOST_LOG_DYN_LINK

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

PylonWrapper::PylonWrapper(std::string camera_name, Operation operation)
  : camera_name_(camera_name), operation_(operation)

{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper constructor";
}

void PylonWrapper::closeForParameterManipulation()
{
  camera_->Close();
}

int64_t
PylonWrapper::getGain()
{
  BOOST_LOG_TRIVIAL(info) << "Fetching parameter: GainValue";
  BOOST_LOG_TRIVIAL(info) <<   "camera.Gain.GetValue()";

  BOOST_LOG_TRIVIAL(info) <<  "Gain Raw: " << camera_->GainRaw.GetValue();
  return  camera_->GainRaw.GetValue();
}

void
PylonWrapper::setGain(int64_t value)
{
  BOOST_LOG_TRIVIAL(info) << "Fetching parameter: GainValue";
  BOOST_LOG_TRIVIAL(info) << "camera_->Gain.SetValue()";

  camera_->GainRaw.SetValue(value);
}

int64_t
PylonWrapper::getShutterTime()
{
  BOOST_LOG_TRIVIAL(info) <<"getShutterTime";
  BOOST_LOG_TRIVIAL(info) << "getShutterTime()";
  BOOST_LOG_TRIVIAL(info) <<"X1";

  int exposureTimeRaw = 1;//camera_->ExposureTimeRaw.GetValue();
  BOOST_LOG_TRIVIAL(info) <<"X221";

  float exposureTimeAbs = camera_->ExposureTimeAbs.GetValue();
  BOOST_LOG_TRIVIAL(info) <<"X14";
  //int exposureTimeRaw2 = camera_->ExposureTimeRaw.GetValue();
  BOOST_LOG_TRIVIAL(info) <<"X12";
  return (int)((exposureTimeAbs * exposureTimeRaw)+0.5);
}

void
PylonWrapper::setShutterTime(int64_t value)
{
  BOOST_LOG_TRIVIAL(info) << "SetShutterTime(" << value <<")";
  camera_->ExposureTimeAbs.SetValue(1);
  camera_->ExposureTimeRaw.SetValue(value);
}

void
PylonWrapper::setRegion(PlanarRegion region)
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::setRegion()";

  camera_->Width.SetValue(region.size_x);
  camera_->Height.SetValue(region.size_y);

  camera_->OffsetX.SetValue(region.offset_x);
  camera_->OffsetY.SetValue(region.offset_y);
}


PlanarRegion
PylonWrapper::getRegion()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::getRegion()";

  int64_t height = camera_->Height.GetValue();

  int64_t i = camera_->Width.GetValue();

  int64_t width = camera_->Width.GetValue();
  int64_t height2 = camera_->Height.GetValue();

  int64_t offsetX = camera_->OffsetX.GetValue();
  int64_t offsetY = camera_->OffsetY.GetValue();

  BOOST_LOG_TRIVIAL(info) << "width: "<< width << " " << "height: " << height;
  BOOST_LOG_TRIVIAL(info) << "offsetX: "<< offsetX << " " << "offsetY: " << offsetY;

  PlanarRegion region;
  region.size_x = width;
  region.size_y = height;

  region.offset_x = offsetX;
  region.offset_y = offsetY;
  return region;
}

void
PylonWrapper::setRegionEnabled(bool regionEnabled)
{
  BOOST_LOG_TRIVIAL(info) << "setRegionEnabled()";
  //camera_->GainRaw.SetValue(value);
}

bool
PylonWrapper::getRegionEnabled()
{
  BOOST_LOG_TRIVIAL(info) << "getRegionEnabled()";
  return camera_->GainRaw.GetValue();
}

bool
PylonWrapper::checkTriggerInterval(int64_t period)
{
  BOOST_LOG_TRIVIAL(info) << "checkTriggerInterval(" << period << ")";

  /* My understanding:
   * if one sets AcquisitionFrameRateAbs Then ResultingFrameRateAbs will give you the what rate you will get
   *
   * Algorithm:
   * 1. Remember old AcquisitionFrameRateAbs
   * 2. Set Wanted value for  AcquisitionFrameRateAbs
   * 3. Read out ResultingFrameRateAbs
   * 4. Set Back old AcquisitionFrameRateAbs
   * 5. Check if ResultingFrameRateAbs is close to AcquisitionFrameRateAbs, then ok
   * (Remember Camera is operating i Hertz second (float) , but we get inn period in i us int64 )
   */

  float wished_rate_in_Hz = 1.e6/period;// Convert to Hz
  // Must be enabled to do calculating
  //1. Stored in  sampleRate_in_Hz_

  camera_->AcquisitionFrameRateEnable.SetValue(true);
  // 2.
  BOOST_LOG_TRIVIAL(info) << "Testing frame rate: " << wished_rate_in_Hz << "Hz";
  try
    {
      camera_->AcquisitionFrameRateAbs.SetValue(wished_rate_in_Hz);
    }
  catch (GenICam::GenericException &e)
    {
      // Error handling.
      BOOST_LOG_TRIVIAL(info)  << "An exception occurred." << e.GetDescription();
      BOOST_LOG_TRIVIAL(info) << "frame rate is out of range: " << wished_rate_in_Hz;
      return false;
    }
  //3.
  float resulting_rate_in_Hz = camera_->ResultingFrameRateAbs.GetValue();
  BOOST_LOG_TRIVIAL(info) << "Reading Resulting frame rate  (ResultingFrameRateAbs)"
                          << resulting_rate_in_Hz << "Hz";
  //4.
  BOOST_LOG_TRIVIAL(info) << "Setting back old sample rate";
  camera_->AcquisitionFrameRateAbs.SetValue(sample_rate_in_Hz_);
  //.5
  if (abs(wished_rate_in_Hz - resulting_rate_in_Hz) < 1)
    {
      sample_rate_in_Hz_= resulting_rate_in_Hz;
      BOOST_LOG_TRIVIAL(info) << "Test of sample rate yields ok. Storing it";
      return true;
    }
  else
    {
      BOOST_LOG_TRIVIAL(info) << "Test of sample rate NOT yields ok. not keeping it";
      return false;
    }
}

int64_t
PylonWrapper::getMaxShutterTime()
{
  BOOST_LOG_TRIVIAL(info) << "getMaxShutterTime()";
  return camera_->MaxShutterTime.GetValue();
}

void
PylonWrapper::setMaxShutterTime(int64_t value)
{
  BOOST_LOG_TRIVIAL(info) << "setMaxShutterTime(" << value << ")";
  camera_->MaxShutterTime.SetValue(value);
}

bool
PylonWrapper::getAutoExposureEnabled()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::Fetching parameter: getAutoExposureEnabled";

  bool retval = false;
  Basler_GigECamera::ExposureAutoEnums e = camera_->ExposureAuto.GetValue();

  /*
  ExposureAuto_Off --> Disables the Exposure Auto function.
  ExposureAuto_Once -->	Sets operation mode to 'once'.
  ExposureAuto_Continuous --> Sets operation mode to 'continuous'.
  */

  switch (e)
    {
    case Basler_GigECamera::ExposureAuto_Continuous:
      BOOST_LOG_TRIVIAL(info) << "Fetched parameter: ExposureAuto_Continuous";
      retval = true;
      break;

    case Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Once:
      BOOST_LOG_TRIVIAL(info) << "Fetched parameter: :ExposureAuto_Once ";
      retval = false;
      break;

    case Basler_GigECamera::ExposureAutoEnums:: ExposureAuto_Off:
      BOOST_LOG_TRIVIAL(info) << "Fetched parameter:ExposureAuto_Off";
      retval = false;
      break;
    }
  return retval;
}

// TODO What about once?
void
PylonWrapper::setAutoExposureEnabled(bool enabled)
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::setAutoExposureEnabled(AutoExposure): " << enabled;
  if (enabled)
    {
      camera_->ExposureAuto.SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Continuous);
    }
  else
    {
      camera_->ExposureAuto.SetValue(Basler_GigECamera::ExposureAutoEnums::ExposureAuto_Off);
    }

}

void
PylonWrapper::do_activate()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::do_activate()";
}

void
PylonWrapper::do_deactivate()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::do_deactivate()";
}

void
PylonWrapper::do_start()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::do_start():" << free_running_;
  if (free_running_)
    {

      camera_->TriggerMode.SetValue(Basler_GigECamera::TriggerModeEnums::TriggerMode_Off);
      camera_->AcquisitionFrameRateEnable.SetValue(true);
      BOOST_LOG_TRIVIAL(info) << "PylonWrapper::do_start() sample_rate_in_Hz_:" << sample_rate_in_Hz_;
      camera_->AcquisitionFrameRateAbs.SetValue(sample_rate_in_Hz_);

    }
  else
    {
      camera_->AcquisitionFrameRateEnable.SetValue(false);
      camera_->TriggerMode.SetValue(Basler_GigECamera::TriggerModeEnums::TriggerMode_On);
      camera_->TriggerSource.SetValue(Basler_GigECamera::TriggerSourceEnums::TriggerSource_Line1);
      camera_->TriggerSelector.SetValue(Basler_GigECamera::TriggerSelectorEnums::TriggerSelector_FrameStart);

    }
  startSampling();
}

void
PylonWrapper::connect()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::connect()";
  BOOST_LOG_TRIVIAL(info) << "Creating Camera...";

  try
    {
      CDeviceInfo info;

      info.SetUserDefinedName("i3ds-basler-hr");
      info.SetDeviceClass(Pylon::CBaslerGigEInstantCamera::DeviceClass());

      camera_ = std::make_unique<Pylon::CBaslerGigEInstantCamera>(CTlFactory::GetInstance().CreateFirstDevice(info));

      BOOST_LOG_TRIVIAL(info) << "Camera Created.";

      //Print the model name of the camera.
      BOOST_LOG_TRIVIAL(info) << "Using device:  " << camera->GetDeviceInfo().GetModelName();
      BOOST_LOG_TRIVIAL(info) << "Friendly name: " << camera->GetDeviceInfo().GetFriendlyName();
      BOOST_LOG_TRIVIAL(info) << "Full name:     " << camera->GetDeviceInfo().GetFullName();
      BOOST_LOG_TRIVIAL(info) << "SerialNumber:  " << camera->GetDeviceInfo().GetSerialNumber();

      camera_->Open();

      //Switching format
      camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono12);

      // Setting to a reasonable frequency.
      // the preprogrammed sample frequency of 410 Hz will make the system unstable.
      camera_->AcquisitionFrameRateAbs.SetValue(1.0);

      sample_rate_in_Hz_ = camera_->AcquisitionFrameRateAbs.GetValue();

    }
  catch (GenICam::GenericException &e)
    {
      // Error handling.
      std::ostringstream errorDescription;
      errorDescription << "An exception occurred." << e.GetDescription();
      throw i3ds::CommandError(error_value, errorDescription.str());;
    }
}

void
PylonWrapper::do_stop()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::do_stop()";
  stopSampling();
}

void
PylonWrapper::stopSampling()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::stopSampling()";
  camera_->StopGrabbing();
  if (threadSamplingLoop.joinable())
    {
      threadSamplingLoop.join();
    }
}

void
PylonWrapper::startSamplingLoop()
{
  BOOST_LOG_TRIVIAL(info) << "PylonWrapper::startSamplingLoop()";

  // The parameter MaxNumBuffer can be used to control the count of buffers
  // allocated for grabbing. The default value of this parameter is 10.
  camera_->MaxNumBuffer = 10;

  // create pylon image format converter and pylon image
  CImageFormatConverter formatConverter;
  formatConverter.OutputPixelFormat = PixelType_BGR8packed;
  CPylonImage pylonImage;

  // Start the grabbing of c_countOfImagesToGrab images.
  // The camera device is parameterized with a default configuration which
  // sets up free-running continuous acquisition.
  //camera_->StartGrabbing ();
  //  camera_->StartGrabbing (c_countOfImagesToGrab);

  float AcquisitionFrameRateAbsFloat = camera_->AcquisitionFrameRateAbs.GetValue();
  BOOST_LOG_TRIVIAL(info) << "AcquisitionFrameRateAbsFloat: " << AcquisitionFrameRateAbsFloat << "Hz="
                          << (1./AcquisitionFrameRateAbsFloat) << "sec";


  float timeoutGrabingFloat =  2.0*1000./camera_->ResultingFrameRateAbs.GetValue() ;
  BOOST_LOG_TRIVIAL(info) << "timeoutGrabingFloat: " << timeoutGrabingFloat;
  int  timeoutGrabingInt = ceil(timeoutGrabingFloat);
  BOOST_LOG_TRIVIAL(info) << "timeoutGrabingInt: " << timeoutGrabingInt << "ms";
  camera_->StartGrabbing();

  // This smart pointer will receive the grab result data.
  CGrabResultPtr ptrGrabResult;

  // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
  // when c_countOfImagesToGrab images have been retrieved.
  while (camera_->IsGrabbing())
    {
      // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
      camera_->RetrieveResult(timeoutGrabingInt,
                              ptrGrabResult,
                              TimeoutHandling_ThrowException);

      // Image grabbed successfully?
      if (ptrGrabResult->GrabSucceeded())
        {
          // Access the image data.
          int width = ptrGrabResult->GetWidth();
          int height = ptrGrabResult->GetHeight();

          BOOST_LOG_TRIVIAL(info) << "SizeX: " << width;
          BOOST_LOG_TRIVIAL(info) << "SizeY: " << height;

          const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
          BOOST_LOG_TRIVIAL(info) << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0];

          operation_(pImageBuffer, width, height);
        }
      else
        {
          BOOST_LOG_TRIVIAL(info) << "Error: " << ptrGrabResult->GetErrorCode() << " "
                                  << ptrGrabResult->GetErrorDescription();
        }
    }
}

void
PylonWrapper::startSampling()
{

  BOOST_LOG_TRIVIAL(info) << "startSampling()";

  try
    {
      threadSamplingLoop = std::thread(&PylonWrapper::startSamplingLoop, this);
    }
  catch (GenICam::GenericException &e)
    {
      // Error handling.
      cerr << "An exception occurred." << endl
           << e.GetDescription() << endl;
    }
}
