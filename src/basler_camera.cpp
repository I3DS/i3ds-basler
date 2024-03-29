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

i3ds::BaslerCamera::BaslerCamera(Context::Ptr context, i3ds_asn1::NodeID node, Parameters param, bool enable_trigger_output)
  : GigECamera(context, node, param),
    enable_trigger_output_(enable_trigger_output),
    black_override_(false),
    black_(0),
    gamma_override_(false),
    gamma_enable_(true),
    gamma_(1.0)
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

  camera_ = nullptr;

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

      // Close camera if error is later than this.
      try
        {
          // Set streaming options.
          camera_->GevSCPSPacketSize.SetValue(param_.packet_size);
          camera_->GevSCPD.SetValue(param_.packet_delay);

          // Set pixel format depending on data depth.
          switch (param_.frame_mode)
            {
              case i3ds_asn1::Frame_mode_t_mode_mono:
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
                    BOOST_LOG_TRIVIAL(error) << "Unsupported data depth for mono images: " << param_.data_depth;
                  }
                break;
              case i3ds_asn1::Frame_mode_t_mode_rgb:
                switch (param_.data_depth)
                  {
                  case 12:
                    BOOST_LOG_TRIVIAL(info) << "Pixel format: RGB12";
                    camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_RGB12Packed);
                    break;

                  case 8:
                    BOOST_LOG_TRIVIAL(info) << "Pixel format: RGB8";
                    camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_RGB8Packed);
                    break;

                  default:
                    BOOST_LOG_TRIVIAL(error) << "Unsupported data depth for RGB images: " << param_.data_depth;
                  }

		  // Select auto function ROI 2
		  camera_->AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI2);
		  // Enable the Balance White Auto auto function
		  // for the auto function ROI selected
		  camera_->AutoFunctionAOIUsageWhiteBalance.SetValue(true);
		  // Enable Balance White Auto by setting the operating mode to Continuous
		  camera_->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);

                break;
              case i3ds_asn1::Frame_mode_t_mode_uyvy:
                  BOOST_LOG_TRIVIAL(info) << "Pixel format: YUV422";
                  camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_YUV422Packed);

		  // Select auto function ROI 2
		  camera_->AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI2);
		  // Enable the Balance White Auto auto function
		  // for the auto function ROI selected
		  camera_->AutoFunctionAOIUsageWhiteBalance.SetValue(true);
		  // Enable Balance White Auto by setting the operating mode to Continuous
		  camera_->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);

                  break;
              default:
                BOOST_LOG_TRIVIAL(error) << "Unsupported frame-mode: " << param_.frame_mode;

            }
          if (enable_trigger_output_)
            {
              enableTriggerOutput();
            }

          if (gamma_override_) {
              camera_->GammaEnable.SetValue(gamma_enable_);
              if (gamma_enable_) {
                  camera_->GammaSelector.SetValue(GammaSelector_User);
                  camera_->Gamma.SetValue(gamma_);
                  if (!black_override_) {
                      // User has no opinin, set black to 0.0 as per basler doc
                      // https://docs.baslerweb.com/gamma.html
                      camera_->BlackLevelRaw.SetValue(0.0);
                      BOOST_LOG_TRIVIAL(info) << "Gamma enable forces new black value, setting to 0.0. (Override/change with --black-level <val>)";
                  }
              }

              BOOST_LOG_TRIVIAL(info) << "GammaEnable: " << camera_->GammaEnable.GetValue();
              BOOST_LOG_TRIVIAL(info) << "GammaSelector: " << camera_->GammaSelector.GetValue();
              BOOST_LOG_TRIVIAL(info) << "Gamma: " << camera_->Gamma.GetValue();
              if (!black_override_)
                  BOOST_LOG_TRIVIAL(info) << "BlackLevelRaw: " << camera_->BlackLevelRaw.GetValue();
          }

          if (black_override_) {
              camera_->BlackLevelRaw.SetValue(black_);
              BOOST_LOG_TRIVIAL(info) << "BlackLevelRaw: " << camera_->BlackLevelRaw.GetValue();
          }
        }
      catch (GenICam::GenericException &e)
        {
          if (camera_)
            {
              camera_->Close();
            }
          throw;
        }
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();

      throw i3ds::CommandError(i3ds_asn1::ResultCode_error_other, "Error connecting to camera: " + std::string(e.what()));
    }
}

void
i3ds::BaslerCamera::Close()
{
  BOOST_LOG_TRIVIAL(info) << "Close()";

  if (sampler_.joinable())
    {
      sampler_.join();
    }

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
          BOOST_LOG_TRIVIAL(debug) << "Setting external trigger values";

          camera_->AcquisitionFrameRateEnable.SetValue(false);

          camera_->TriggerSelector.SetValue(TriggerSelector_FrameStart);
          camera_->TriggerMode.SetValue(TriggerMode_On);
          camera_->TriggerSource.SetValue(TriggerSource_Line1);
          camera_->ExposureMode.SetValue(ExposureMode_Timed);
        }
      else
        {
          BOOST_LOG_TRIVIAL(debug) << "Setting internal trigger values";

          camera_->TriggerMode.SetValue(TriggerMode_Off);
          camera_->AcquisitionFrameRateEnable.SetValue(true);
          camera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
          camera_->AcquisitionFrameRateAbs.SetValue(1.0e6 / period());
        }
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();
      set_error_state("Error starting camera: " + std::string(e.what()), false);
    }

  sampler_ = std::thread(&i3ds::BaslerCamera::SampleLoop, this);
}

void
i3ds::BaslerCamera::Stop()
{
  BOOST_LOG_TRIVIAL(debug) << "Stop()";

  try
    {
      camera_->StopGrabbing();
    }
  catch (GenICam::GenericException &e)
    {
      BOOST_LOG_TRIVIAL(warning) << e.what();
      if (sampler_.joinable())
        {
          sampler_.join();
        }
      set_error_state("Error stopping camera: " + std::string(e.what()), false);
    }

  if (sampler_.joinable())
    {
      sampler_.join();
    }
}

bool
i3ds::BaslerCamera::setInternalTrigger(int64_t period_us)
{
  BOOST_LOG_TRIVIAL(debug) << "setInternalTrigger()";

  /* Algorithm for testing if framerate is supported :
     * if one sets AcquisitionFrameRateAbs, then ResultingFrameRateAbs will give you the what rate you acctually will get
     *
     * Algorithm:
     * 1. Store old AcquisitionFrameRateAbs
     * 2. Set Wanted value for  AcquisitionFrameRateAbs
     * 3. Read out ResultingFrameRateAbs
     * 4. Set Back old AcquisitionFrameRateAbs
     * 5. Check if ResultingFrameRateAbs is close to AcquisitionFrameRateAbs, then ok
     * (Remember Camera is operating i Hertz second (float) , but we get inn period in i us int64 )
     */

  const float wanted_rate_in_Hz = 1.e6 / period_us;// Convert to Hz

  // Must be enabled to do calculating
  camera_->AcquisitionFrameRateEnable.SetValue(true);

  // 1. Remember the old value
  const float old_sample_rate_in_Hz = camera_->AcquisitionFrameRateAbs.GetValue();

  // 2.
  BOOST_LOG_TRIVIAL(trace) << "Testing frame rate: " << wanted_rate_in_Hz << "Hz";
  try
    {
      camera_->AcquisitionFrameRateAbs.SetValue(wanted_rate_in_Hz);
    }
  catch (GenICam::GenericException &e)    // Error handling.
    {
      BOOST_LOG_TRIVIAL(trace)  << "An exception occurred." << e.GetDescription();
      BOOST_LOG_TRIVIAL(trace) << "frame rate is probably out of range: " << wanted_rate_in_Hz;
      return false;
    }

  //3.
  const float resulting_rate_in_Hz = camera_->ResultingFrameRateAbs.GetValue();
  BOOST_LOG_TRIVIAL(trace) << "Reading resulting frame rate  (ResultingFrameRateAbs)"
                           << resulting_rate_in_Hz << "Hz";

  //4.
  //BOOST_LOG_TRIVIAL(trace) << "Setting back old sample rate";
  //camera_->AcquisitionFrameRateAbs.SetValue(old_sample_rate_in_Hz);

  //.5 Accepting new rate if result is within 1Hz of wanted_frequency
  if (abs(wanted_rate_in_Hz - resulting_rate_in_Hz) < 1.)
    {
      BOOST_LOG_TRIVIAL(trace) << "Test of sample rate yields OK. Storing and using it";
      return true;
    }
  else
    {
      BOOST_LOG_TRIVIAL(trace) << "Test of sample rate NOT OK. Do not keep it";
      BOOST_LOG_TRIVIAL(trace) << "Setting back old sample rate";
      camera_->AcquisitionFrameRateAbs.SetValue(old_sample_rate_in_Hz);

      return false;
    }
}

int64_t
i3ds::BaslerCamera::getSensorWidth() const
{
  BOOST_LOG_TRIVIAL(debug) << "getSensorWidth()";

  return camera_->SensorWidth.GetValue();
}

int64_t
i3ds::BaslerCamera::getSensorHeight() const
{
  BOOST_LOG_TRIVIAL(debug) << "getSensorHeight()";

  return camera_->SensorHeight.GetValue();
}

bool
i3ds::BaslerCamera::isRegionSupported() const
{
  BOOST_LOG_TRIVIAL(debug) << "isRegionSupported()";

  return true;
}

int64_t
i3ds::BaslerCamera::getRegionWidth() const
{
  BOOST_LOG_TRIVIAL(trace) << "getRegionWidth()";

  return camera_->Width.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionHeight() const
{
  BOOST_LOG_TRIVIAL(trace) << "getRegionHeight()";

  return camera_->Height.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionOffsetX() const
{
  BOOST_LOG_TRIVIAL(trace) << "getRegionOffsetX()";

  return camera_->OffsetX.GetValue();
}

int64_t
i3ds::BaslerCamera::getRegionOffsetY() const
{
  BOOST_LOG_TRIVIAL(trace) << "getRegionOffsetY()";

  return camera_->OffsetY.GetValue();
}

void
i3ds::BaslerCamera::setRegionWidth(int64_t width)
{
  BOOST_LOG_TRIVIAL(debug) << "setRegionWidth()";

  camera_->Width.SetValue(width);
}

void
i3ds::BaslerCamera::setRegionHeight(int64_t height)
{
  BOOST_LOG_TRIVIAL(debug) << "setRegionHeight()";

  camera_->Height.SetValue(height);
}

void
i3ds::BaslerCamera::setRegionOffsetX(int64_t offset_x)
{
  BOOST_LOG_TRIVIAL(debug) << "setRegionOffsetX()";

  camera_->OffsetX.SetValue(offset_x);
}

void
i3ds::BaslerCamera::setRegionOffsetY(int64_t offset_y)
{
  BOOST_LOG_TRIVIAL(debug) << "setRegionOffsetY()";

  camera_->OffsetY.SetValue(offset_y);
}

int64_t
i3ds::BaslerCamera::getShutter() const
{
  BOOST_LOG_TRIVIAL(debug) << "getShutter()";

  return camera_->ExposureTimeAbs.GetValue();
}

int64_t
i3ds::BaslerCamera::getMaxShutter() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMaxShutter()";

  return camera_->ExposureTimeAbs.GetMax();
}

int64_t
i3ds::BaslerCamera::getMinShutter() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMinShutter()";

  return camera_->ExposureTimeAbs.GetMin();
}

void
i3ds::BaslerCamera::setShutter(int64_t shutter_us)
{
  BOOST_LOG_TRIVIAL(debug) << "setShutter()";

  camera_->ExposureTimeAbs.SetValue(shutter_us);
}

bool
i3ds::BaslerCamera::isAutoShutterSupported() const
{
  BOOST_LOG_TRIVIAL(debug) << "isAutoShutterSupported()";

  return true;
}

bool
i3ds::BaslerCamera::getAutoShutterEnabled() const
{
  BOOST_LOG_TRIVIAL(debug) << "getAutoShutterEnabled()";

  return camera_->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Continuous;
}

void
i3ds::BaslerCamera::setAutoShutterEnabled(bool enable)
{
  BOOST_LOG_TRIVIAL(debug) << "setAutoShutterEnabled()";

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
  BOOST_LOG_TRIVIAL(debug) << "getAutoShutterLimit()";

  return camera_->AutoExposureTimeAbsUpperLimit.GetValue();
}

int64_t
i3ds::BaslerCamera::getMaxAutoShutterLimit() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMaxAutoShutterLimit()";

  return camera_->AutoExposureTimeAbsUpperLimit.GetMax();
}

int64_t
i3ds::BaslerCamera::getMinAutoShutterLimit() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMinAutoShutterLimit()";

  return camera_->AutoExposureTimeAbsUpperLimit.GetMin();
}

void
i3ds::BaslerCamera::setAutoShutterLimit(int64_t shutter_limit)
{
  BOOST_LOG_TRIVIAL(debug) << "setAutoShutterLimit()";

  const double lower = camera_->AutoExposureTimeAbsLowerLimit.GetMin();
  const double upper = shutter_limit;

  camera_->AutoExposureTimeAbsLowerLimit.SetValue(lower);
  camera_->AutoExposureTimeAbsUpperLimit.SetValue(upper);
}

double
i3ds::BaslerCamera::getGain() const
{
  BOOST_LOG_TRIVIAL(debug) << "getGain()";

  return raw_to_gain(camera_->GainRaw.GetValue());
}

double
i3ds::BaslerCamera::getMaxGain() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMaxGain()";

  return raw_to_gain(camera_->GainRaw.GetMax());
}

double
i3ds::BaslerCamera::getMinGain() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMinGain()";

  return raw_to_gain(camera_->GainRaw.GetMin());
}

void
i3ds::BaslerCamera::setGain(double gain)
{
  BOOST_LOG_TRIVIAL(debug) << "setGain()";

  camera_->GainRaw.SetValue(gain_to_raw(gain));
}

bool
i3ds::BaslerCamera::isAutoGainSupported() const
{
  BOOST_LOG_TRIVIAL(debug) << "isAutoGainSupported()";

  // TODO: Handle that some cameras don't support auto gain and shutter at same time.

#ifdef SCOUT
  // TODO: Scout camera has no autogain
  return false;
#else
  return true; // Normally used. Used for basler HR camera
#endif

}

bool
i3ds::BaslerCamera::getAutoGainEnabled() const
{
  BOOST_LOG_TRIVIAL(debug) << "getAutoGainEnabled()";

  return camera_->GainAuto.GetValue() == GainAuto_Continuous;
}

void
i3ds::BaslerCamera::setAutoGainEnabled(bool enable)
{
  BOOST_LOG_TRIVIAL(debug) << "setAutoGainEnabled()";

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
  BOOST_LOG_TRIVIAL(debug) << "getAutoGainLimit()";

  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetValue());
}

double
i3ds::BaslerCamera::getMaxAutoGainLimit() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMaxAutoGainLimit()";

  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetMax());
}

double
i3ds::BaslerCamera::getMinAutoGainLimit() const
{
  BOOST_LOG_TRIVIAL(debug) << "getMinAutoGainLimit()";

  return raw_to_gain(camera_->AutoGainRawUpperLimit.GetMin());
}

void
i3ds::BaslerCamera::setAutoGainLimit(double gain_limit)
{
  BOOST_LOG_TRIVIAL(debug) << "setAutoGainLimit()";
  const int64_t lower = camera_->AutoGainRawLowerLimit.GetMin();
  // Setting lower limit low first such that one does not get a exception if the lower limit is higher than requested high limit.
  camera_->AutoGainRawLowerLimit.SetValue(lower);
  const int64_t upper = gain_to_raw(gain_limit);
  camera_->AutoGainRawUpperLimit.SetValue(upper);
}

double
i3ds::BaslerCamera::raw_to_gain(int64_t raw) const
{
  // TODO: Camera specific for ACE acA2040-25gmNIR.
  return 20.0 * log10((1.0 / 32.0) * raw);
}

int64_t
i3ds::BaslerCamera::gain_to_raw(i3ds_asn1::SensorGain gain) const
{
  // TODO: Camera specific for ACE acA2040-25gmNIR.
  return (int64_t) 32.0 * pow(10, gain / 20.0);
}


void
i3ds::BaslerCamera::set_error_state(const std::string &error_message, const bool dont_throw = false )
{
  BOOST_LOG_TRIVIAL(error) << "set_error_state: Error message: " << error_message;

  set_failure();

  if (!dont_throw)
    {
      throw i3ds::CommandError(i3ds_asn1::ResultCode_error_other, error_message);
    }
}


void
i3ds::BaslerCamera::SampleLoop()
{
  BOOST_LOG_TRIVIAL(debug) << "SampleLoop()";

  // The parameter MaxNumBuffer can be used to control the count of buffers
  // allocated for grabbing. The default value of this parameter is 10.
  camera_->MaxNumBuffer = 10;

  // create pylon image
  Pylon::CPylonImage pylonImage;

  // This smart pointer will receive the grab result data.
  Pylon::CGrabResultPtr ptrGrabResult;

  // Timeout 4 times a second. The only thing we are waiting for is actually a stopline.
  // Else we always are waiting for a image.
  const int timeout_ms = 2 * period() / 1000;
  const int max_timeouts = 10;

  const int max_RetriveResult_errors = 10;
  const int max_grab_errors = 10;

  retrive_errors_ = 0;
  timeout_counter_ = 0;
  grab_errors_ = 0;

  // Start the grabbing of images.
  camera_->StartGrabbing();

  while (camera_->IsGrabbing())
    {
      try
        {
          // Wait for an image and then retrieve it.
          bool retrive_success = camera_->RetrieveResult(timeout_ms, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

          if (!retrive_success)
            {
              BOOST_LOG_TRIVIAL(warning) << "RetrieveResult error";

              retrive_errors_ ++;

              if (retrive_errors_ > max_RetriveResult_errors)
                {
                  BOOST_LOG_TRIVIAL(warning) << "max_RetriveResult_error. Going to failstate!";
                  set_error_state("Too many RetriveResult() errors. Going to failstate!", true);
                }

              break;
            }

          // Image grabbed successfully?
          if (ptrGrabResult->GrabSucceeded())
            {
              // Access the image data.
              const int width = ptrGrabResult->GetWidth();
              const int height = ptrGrabResult->GetHeight();

              const i3ds_asn1::byte* pImageBuffer = (const i3ds_asn1::byte*) ptrGrabResult->GetBuffer();

              send_sample(pImageBuffer, width, height);
              if (image_sequence_enabled() && !image_sequence_ready_next()) {
                  camera_->StopGrabbing();
              }

              retrive_errors_ = 0;
              timeout_counter_ = 0;
              grab_errors_ = 0;
            }
          else
            {
              BOOST_LOG_TRIVIAL(warning) << "Error grabbing: "
                                         << ptrGrabResult->GetErrorCode() << " "
                                         << ptrGrabResult->GetErrorDescription();

              grab_errors_ ++;

              if (grab_errors_ > max_grab_errors)
                {
                  BOOST_LOG_TRIVIAL(warning) << "Too many grab errors in sample loop. Going to failstate!";
                  set_error_state("Too many grab errors in sample loop. Going to failstate!", true);
                  break;
                }
            }
        }
      catch (Pylon::TimeoutException& e)
        {
          BOOST_LOG_TRIVIAL( debug ) << "Normal timeout one sample!";

          timeout_counter_ ++;

          if (timeout_counter_ > max_timeouts)
            {
              BOOST_LOG_TRIVIAL(warning) << "Too many timeout sample errors. Going to failstate!";
              set_error_state("Too many timeout sample errors. Going to failstate!", true );
              break;
            }
        }
      catch (GenICam::GenericException& e)
        {
          BOOST_LOG_TRIVIAL(warning) << e.what();
          set_error_state("Exception error in sample loop: " + std::string ( e.what() ), true );

          break;
        }
    }
}


void
i3ds::BaslerCamera::enableTriggerOutput()
{
  BOOST_LOG_TRIVIAL(info) << "Enabling trigger output";
  camera_->LineSelector.SetValue(LineSelector_Out1);
  camera_->LineSource.SetValue(LineSource_ExposureActive);
  BOOST_LOG_TRIVIAL(info) << "Trigger output enabled";
}

void i3ds::BaslerCamera::overrideBlack(int black)
{
    if (black < 0 || black > 255)
        throw std::invalid_argument("black level out of range [0..255]");

    black_override_ = true;
    black_ = black;
    BOOST_LOG_TRIVIAL(info) << "Adjusting black-level: " << black_;
}

void
i3ds::BaslerCamera::overrideGamma(bool gamma_enable, double gamma)
{
    if (gamma < 0.0 || gamma > 4.0)
        throw std::invalid_argument("gamma out of range [0.0..4.0]");

    gamma_override_ = true;
    gamma_enable_ = gamma_enable;
    gamma_ = gamma;
    BOOST_LOG_TRIVIAL(info) << "Adjusting gamma: " << gamma_ << ", enable: " << gamma_enable_;
}
