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


i3ds::BaslerCamera::BaslerCamera(Context::Ptr context, NodeID node, Parameters param, TriggerClient::Ptr trigger)
  : Camera(node),
    param_(param),
    publisher_(context, node),
    trigger_(trigger)
{
  flash_enabled_ = false;
  flash_strength_ = 0.0;

  pattern_enabled_ = false;
  pattern_sequence_ = 0;

  camera_ = nullptr;

  if (trigger_)
    {
      // Only wait 100 ms for trigger service.
      trigger_->set_timeout(100);
    }

  flash_configurator_ = std::unique_ptr<SerialCommunicator>(new SerialCommunicator(param.wa_flash_port.c_str()));

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
  return (bool)(camera_->ExposureAuto.GetValue() == Basler_GigECamera::ExposureAuto_Continuous);
}

ShutterTime
i3ds::BaslerCamera::max_shutter() const
{
  // TODO: Check need for convertion.
  return (ShutterTime) camera_->AutoExposureTimeAbsUpperLimit.GetValue();
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

bool
i3ds::BaslerCamera::auto_gain_enabled() const
{
  return (camera_->GainAuto.GetValue() == GainAuto_Continuous);
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

      BOOST_LOG_TRIVIAL(info) << "param_.packet_size: " << param_.packet_size;
      BOOST_LOG_TRIVIAL(info) << "packet_delay: " << param_.packet_delay;

      // Set streaming options.
      camera_->GevSCPSPacketSize.SetValue(param_.packet_size);
      camera_->GevSCPD.SetValue(param_.packet_delay);

      // Switching format
      camera_->PixelFormat.SetValue(Basler_GigECamera::PixelFormat_Mono12);

      // Set default sampling to reasonable value.
      if (trigger_)
        {
          set_trigger(param_.camera_output, param_.camera_offset);
        }
      else
        {
          camera_->AcquisitionFrameRateEnable.SetValue(true);
          camera_->AcquisitionFrameRateAbs.SetValue(1.0e6 / period());
        }

      // Force it in auto_exposure mode when starting up
      int64_t max_auto_shutter = camera_->AutoExposureTimeAbsUpperLimit.GetMax();
      int64_t max_auto_gain = raw_to_gain(camera_->AutoGainRawUpperLimit.GetMax());

      handle_auto_exposure_helper(max_auto_shutter, max_auto_gain);


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

  if (trigger_)
    {
      camera_->AcquisitionFrameRateEnable.SetValue(false);

      camera_->TriggerSelector.SetValue(TriggerSelector_FrameStart);
      camera_->TriggerMode.SetValue(TriggerMode_On);
      camera_->TriggerSource.SetValue(TriggerSource_Line1);
      camera_->ExposureMode.SetValue(ExposureMode_Timed);

      trigger_->set_generator(param_.trigger_source, period());
      trigger_->enable_channels(trigger_outputs_);
    }
  else
    {
      camera_->TriggerMode.SetValue(TriggerMode_Off);
      camera_->AcquisitionFrameRateEnable.SetValue(true);
      camera_->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
      camera_->AcquisitionFrameRateAbs.SetValue(1.0e6 / period());

      const double fps = camera_->ResultingFrameRateAbs.GetValue();
      BOOST_LOG_TRIVIAL(info) << "Free running at " << fps << " FPS";
    }

  sampler_ = std::thread(&BaslerCamera::SampleLoop, this);
}

void
i3ds::BaslerCamera::do_stop()
{
  BOOST_LOG_TRIVIAL(info) << "do_stop()";

  if (trigger_)
    {
      trigger_->disable_channels(trigger_outputs_);
    }

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

  flash_enabled_ = false;
  flash_strength_ = 0.0;

  pattern_enabled_ = false;
  pattern_sequence_ = 0;

  trigger_outputs_.clear();
}

bool
i3ds::BaslerCamera::is_sampling_supported(SampleCommand sample)
{
  BOOST_LOG_TRIVIAL(info) << "is_rate_supported() " << sample.period;

  if (trigger_)
    {
      // Minimal period 50 ms (= 20 Hz) and maximal 16.7 seconds for external trigger.
      return 50000 <= sample.period && sample.period <= 16777215;
    }

  /* Algorithm for testing if supported :
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

  const float wanted_rate_in_Hz = 1.e6 / sample.period;// Convert to Hz

  // Must be enabled to do calculating
  camera_->AcquisitionFrameRateEnable.SetValue(true);

  // Remember Old value
  const float old_sample_rate_in_Hz = camera_->AcquisitionFrameRateAbs.GetValue();

  // 2.
  BOOST_LOG_TRIVIAL(info) << "Testing frame rate: " << wanted_rate_in_Hz << "Hz";
  try
    {
      camera_->AcquisitionFrameRateAbs.SetValue(wanted_rate_in_Hz);
    }
  catch (GenICam::GenericException &e)
    {
      // Error handling.
      BOOST_LOG_TRIVIAL(info)  << "An exception occurred." << e.GetDescription();
      BOOST_LOG_TRIVIAL(info) << "frame rate is out of range: " << wanted_rate_in_Hz;
      return false;
    }

  //3.
  const float resulting_rate_in_Hz = camera_->ResultingFrameRateAbs.GetValue();
  BOOST_LOG_TRIVIAL(info) << "Reading Resulting frame rate  (ResultingFrameRateAbs)"
                          << resulting_rate_in_Hz << "Hz";

  //4.
  BOOST_LOG_TRIVIAL(info) << "Setting back old sample rate";
  camera_->AcquisitionFrameRateAbs.SetValue(old_sample_rate_in_Hz);

  //.5 Accepting it if result is within 1Hz of wanted_frequency
  if (abs(wanted_rate_in_Hz - resulting_rate_in_Hz) < 1)
    {
      BOOST_LOG_TRIVIAL(info) << "Test of sample rate yields OK. Storing it";
      return true;
    }
  else
    {
      BOOST_LOG_TRIVIAL(info) << "Test of sample rate NOT OK. Do not keep it";
      return false;
    }
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

  const int64_t shutter = command.request.shutter;
  const int64_t shutter_max = camera_->ExposureTimeRaw.GetMax();
  const int64_t shutter_min = camera_->ExposureTimeRaw.GetMin();

  if (shutter > (int64_t) period())
    {
      throw i3ds::CommandError(error_value, "Shutter time longer than period!");
    }

  if (shutter > shutter_max)
    {
      throw i3ds::CommandError(error_value, "Shutter time longer than " + std::to_string(shutter_max));
    }

  if (shutter < shutter_min)
    {
      throw i3ds::CommandError(error_value, "Shutter time shorter than " + std::to_string(shutter_min));
    }


  if (auto_gain_enabled())
    {
      throw i3ds::CommandError(error_value, "AutoGain not in off mode");
    }

  const int64_t raw = gain_to_raw(command.request.gain);
  const int64_t raw_max = camera_->GainRaw.GetMax();
  const int64_t raw_min = camera_->GainRaw.GetMin();

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
i3ds::BaslerCamera::handle_auto_exposure_helper(const int64_t max_shutter_time, const int64_t max_gain_parameter) const
{
  BOOST_LOG_TRIVIAL(info) << "handle_auto_exposure_helper";

  // Check limits
  const double auto_exposure_max_limit = camera_->AutoExposureTimeAbsUpperLimit.GetMax();
  const double auto_exposure_min_limit = camera_->AutoExposureTimeAbsLowerLimit.GetMin();
  if ((max_shutter_time > auto_exposure_max_limit) || (max_shutter_time < auto_exposure_min_limit))
    {
      throw i3ds::CommandError(error_value, "MaxShutterTime must be within: [" +
                               std::to_string(auto_exposure_min_limit) + "," +
                               std::to_string(auto_exposure_max_limit) + "]");
    }

  const double auto_gain_max_limit_raw = camera_->AutoGainRawUpperLimit.GetMax();
  const double auto_gain_min_limit_raw = camera_->AutoGainRawLowerLimit.GetMin();
  const double min_gain_raw = camera_->AutoGainRawLowerLimit.GetMin();
  const double max_gain_parameter_raw = gain_to_raw(max_gain_parameter);



  if ((max_gain_parameter_raw > auto_gain_max_limit_raw) || (max_gain_parameter_raw < auto_gain_min_limit_raw))
    {
      throw i3ds::CommandError(error_value,
                               "Max_AutoGain must be within: [" +
                               std::to_string(raw_to_gain(auto_gain_min_limit_raw)) + "," +
                               std::to_string(raw_to_gain(auto_gain_max_limit_raw)) + "]");
    }



  camera_->AutoGainRawLowerLimit.SetValue(min_gain_raw);
  camera_->AutoGainRawUpperLimit.SetValue(max_gain_parameter_raw);

  camera_->AutoFunctionProfile.SetValue(AutoFunctionProfile_GainMinimum);
  camera_->GainAuto.SetValue(GainAuto_Continuous);
  camera_->ExposureAuto.SetValue(ExposureAuto_Continuous);

  const double min_auto_exposure_lower_limit = camera_->AutoExposureTimeAbsLowerLimit.GetMin();
  const double max_auto_exposure_upper_limit = max_shutter_time;

  camera_->AutoExposureTimeAbsLowerLimit.SetValue(min_auto_exposure_lower_limit);
  camera_->AutoExposureTimeAbsUpperLimit.SetValue(max_auto_exposure_upper_limit);
}


void
i3ds::BaslerCamera::handle_auto_exposure(AutoExposureService::Data& command)
{

  BOOST_LOG_TRIVIAL(info) << "handle_auto_exposure";

  check_active();

  if (command.request.enable)
    {
      handle_auto_exposure_helper(command.request.max_shutter, command.request.max_gain);
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

  if (command.request.enable)
    {
      const PlanarRegion region = command.request.region;
      BOOST_LOG_TRIVIAL(info) << "handle_region()";

      // Test for limits
      if ((region.size_x == 0) || (region.size_y == 0))
        {
          throw i3ds::CommandError(error_value, "One or more of region parameters are zero");
        }

      if ((region.size_x + region.offset_x) > ((unsigned) camera_->SensorWidth.GetValue()))
        {
          throw i3ds::CommandError(error_value,  std::string("Width + offset.x is larger than maximum width for camera : ") +
                                   std::to_string(region.size_x + region.offset_x) + " < "+
                                   std::to_string(camera_->SensorWidth.GetValue()));
        }

      if ((region.size_y + region.offset_y) > (unsigned) camera_->SensorHeight.GetValue())
        {
          throw i3ds::CommandError(error_other, std::string("Heigth + offset.y is larger than maximum height for camera: ") +
                                   std::to_string(region.size_y + region.offset_y) + " < " +
                                   std::to_string((unsigned) camera_->SensorHeight.GetValue()));
        }

      // Have to do resizing in correct order.(Reduse parameter first, increase later)
      if (region.size_x > (unsigned) camera_->Width.GetValue())
        {
          camera_->OffsetX.SetValue(region.offset_x);
          camera_->Width.SetValue(region.size_x);
        }
      else
        {
          camera_->Width.SetValue(region.size_x);
          camera_->OffsetX.SetValue(region.offset_x);
        }

      if (region.size_y > (unsigned) camera_->Height.GetValue())
        {
          camera_->OffsetY.SetValue(region.offset_y);
          camera_->Height.SetValue(region.size_y);
        }
      else
        {
          camera_->Height.SetValue(region.size_y);
          camera_->OffsetY.SetValue(region.offset_y);
        }
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

  if (!trigger_)
    {
      throw i3ds::CommandError(error_other, "Flash not supported in free-running mode");
    }

  flash_enabled_ = command.request.enable;

  if (command.request.enable)
    {
      flash_strength_ = command.request.strength;


      if (flash_strength_ > 100)
	{
	  throw i3ds::CommandError(error_value, "The flash can not give more than 100%");
	}
      BOOST_LOG_TRIVIAL(info) << "handle_flash()";
      float flash_duration_in_ms;

      if (auto_exposure_enabled())
	{
	  flash_duration_in_ms = camera_->AutoExposureTimeAbsUpperLimit.GetValue() / 1000.;
	}
      else
	{
	  flash_duration_in_ms = camera_->ExposureTimeRaw.GetValue() / 1000.;
	}
      BOOST_LOG_TRIVIAL(info) << "handle_flash()" << flash_duration_in_ms;

      // Upper limit for flash
      if (flash_duration_in_ms > 3)
	{
	  flash_duration_in_ms = 3;
	}

      /// Remark: Err 5 is a out of range warning for one parameter.
      /// It is fixed to valid value.
      /// But, it looks as there is a limit with a relationship between duration and flash strength
      /// http://www.gardasoft.com/downloads/?f=112
      //  Page 13(Read it)
      ///
      //Output brightness		850nm variant    | 			|	White variant
      //			Allowed pulsewidth |Allowed duty cycle 	| Allowed pulse width   |   Allowed duty cycle
      // 0% to  20% 		3ms 			6% 			3ms 			3%
      //21% to  30% 		3ms 			6%	 		2ms 			3%
      //31% to  50% 		3ms 			3% 			2ms 			2%
      //51% to 100% 		2ms 			3% 			1ms 			1%




      flash_configurator_->sendConfigurationParameters (
	  1, 			// Configure strobe output
	  flash_duration_in_ms,	// Pulse width ms
	  0.01, 		// Delay from trigger to pulse in ms(0.01 to 999)
	  flash_strength_	/// Settings in percent
	  ); 			// 5th parameter retrigger delay in ms(optional not used)

      // Enable trigger for flash.
      set_trigger(param_.flash_output, param_.flash_offset);


    }
  else
    {
      // Clear trigger, not enabled when operational.
      clear_trigger(param_.flash_output);
    }
}

void
i3ds::BaslerCamera::handle_pattern(PatternService::Data& command)
{
  BOOST_LOG_TRIVIAL(info) << "handle_pattern()";

  check_standby();

  if (!trigger_)
    {
      throw i3ds::CommandError(error_other, "Pattern not supported in free-running mode");
    }

  pattern_enabled_ = command.request.enable;

  if (command.request.enable)
    {
      // Only support one pattern sequence, not controllable as of now.
      if (command.request.sequence != 1)
        {
          throw i3ds::CommandError(error_value, "Unsupported pattern sequence");
        }

      pattern_sequence_ = command.request.sequence;

      // Enable trigger for flash.
      set_trigger(param_.pattern_output, param_.pattern_offset);
    }
  else
    {
      // Reset pattern sequence to disabled.
      pattern_sequence_ = 0;

      // Clear trigger, not enabled when operational.
      clear_trigger(param_.pattern_output);
    }
}

void
i3ds::BaslerCamera::set_trigger(TriggerOutput channel, TriggerOffset offset)
{
  // Set the channel to fire at offset with 100 us pulse.
  trigger_->set_internal_channel(channel, param_.trigger_source, offset, 100, false);

  // Enable the trigger on do_start.
  trigger_outputs_.insert(channel);
}

void
i3ds::BaslerCamera::clear_trigger(TriggerOutput channel)
{
  // Do not enable the trigger on do_start.
  trigger_outputs_.erase(channel);
}

bool
i3ds::BaslerCamera::send_sample(const byte* image, int width, int height)
{
  BOOST_LOG_TRIVIAL(trace) << "BaslerCamera::send_sample() (" << width << "x" << height << ")" ;

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
