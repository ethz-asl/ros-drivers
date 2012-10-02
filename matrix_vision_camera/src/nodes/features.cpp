

#include <cmath>
#include "features.h"
#include "formats.h"

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

/** @file

 @brief Bluecougar features implementation

 @author Markus Achtelik
 */

using namespace mvIMPACT::acquire;

////////////////////////////////////////////////////////////////
// public methods:
////////////////////////////////////////////////////////////////

/** Constructor
 *
 *  @param camera address of DC1394 camera structure.
 */
Features::Features(Device *cam) :
  cam_(cam), fps_(20.0) // set to something != 0
{
}

/** Query and set all features for newly opened (or reopened) device.
 *
 *  @param newconfig [in,out] configuration parameters, updated
 *         to conform with device restrictions.
 *  @return true if successful
 *
 *  @post feature_set_ initialized, if successful
 *  @post oldconfig_ settings available, if successful
 */
bool Features::initialize(Config *newconfig)
{
  reconfigure(newconfig);
  // save configured values
  oldconfig_ = *newconfig;

  return true;
}

double Features::computeFrameTime()
{
  CameraSettingsBlueDevice bfs(cam_);

  double pixel_clock = static_cast<double> (bfs.pixelClock_KHz.read()) * 1.0e3;
  double width = static_cast<double> (bfs.aoiWidth.read(plMaxValue));
  double height = static_cast<double> (bfs.aoiHeight.read(plMaxValue));
  return (width + 94) * (height + 45) / pixel_clock;
}

bool Features::setFramerate(const double & fps_suggested, double * fps_returned)
{
  const double TRIGGER_PULSE_WIDTH = 100.0 * 1.0e-6; // 100 us

  CameraSettingsBlueDevice common_settings(cam_);

  double exposure_time = static_cast<double>(common_settings.expose_us.read()) * 1.0e-6;
  double pixel_clock = static_cast<double>(common_settings.pixelClock_KHz.read()) * 1.0e3;
  double frame_time = computeFrameTime();
  double fps_max = 1.0 / (frame_time + exposure_time + 2 * TRIGGER_PULSE_WIDTH);
  double fps;

  // assuming that the BlueCougar runs in free running mode, this might be conservative
  ROS_INFO("Timing info: px_clock=%f MHz, frametime=%f ms, fps_max=%f ", pixel_clock*1.0e-6, frame_time*1000, fps_max);

  if (fps_suggested > fps_max)
  {
    ROS_WARN("FPS (%f) > FPS_max (%f), limiting to FPS_max", fps_suggested, fps_max);
    fps = fps_max;
  }
  else
    fps = fps_suggested;

  if (fps_returned)
    *fps_returned = fps;

  // setting the framerate can be quite different for the blueDevice families ...

  if (cam_->family.readS() == "mvBlueCOUGAR")
  {
    CameraSettingsBlueCOUGAR bluecougar_settings(cam_);
    double fps_ret;
    configure(bluecougar_settings.frameRate_Hz, fps_suggested, &fps_ret);
    if (fps_returned)
      *fps_returned = (double)fps_ret;
  }
  else if (cam_->family.readS() == "mvBlueFOX" && cam_->product.readS() != "mvBlueFOX-ML/IGC202dG")
  {      
    IOSubSystemBlueFOX bluefox_IOs(cam_);
    CameraSettingsBlueFOX bluefox_settings(cam_);
      
    // define a HRTC program that results in a define image frequency
    // the hardware real time controller shall be used to trigger an image
    bluefox_settings.triggerSource.write(ctsRTCtrl);
    // when the hardware real time controller switches the trigger signal to
    // high the exposure of the image shall start
    bluefox_settings.triggerMode.write(ctmOnHighLevel); // ctmOnHighLevel

    // error checks
    if (bluefox_IOs.RTCtrProgramCount() == 0)
    {
      // no HRTC controllers available (this never happens for the mvBluecougar)
      ROS_WARN("NO HRTC Controllers available");
      return false;
    }

    RTCtrProgram* pRTCtrlprogram = bluefox_IOs.getRTCtrProgram(0);
    if (!pRTCtrlprogram)
    {
      // this only should happen if the system is short of memory
      ROS_WARN("Short on Memory...");
      return false;
    }

    // start of the program

    // we need 5 steps for the program
    pRTCtrlprogram->setProgramSize(5);

    // wait a certain amount of time to achieve the desired frequency
    int progStep = 0;
    RTCtrProgramStep* pRTCtrlStep = 0;
    pRTCtrlStep = pRTCtrlprogram->programStep(progStep++);
    pRTCtrlStep->opCode.write(rtctrlProgWaitClocks);
    pRTCtrlStep->clocks_us.write(static_cast<int>((1.0 / fps - TRIGGER_PULSE_WIDTH) * 1.0e6));

    // trigger an image
    pRTCtrlStep = pRTCtrlprogram->programStep(progStep++);
    pRTCtrlStep->opCode.write(rtctrlProgTriggerSet);

    // high time for the trigger signal (should not be smaller than 100 us)
    pRTCtrlStep = pRTCtrlprogram->programStep(progStep++);
    pRTCtrlStep->opCode.write(rtctrlProgWaitClocks);
    pRTCtrlStep->clocks_us.write(static_cast<int>(TRIGGER_PULSE_WIDTH * 1.0e6));

    // end trigger signal
    pRTCtrlStep = pRTCtrlprogram->programStep(progStep++);
    pRTCtrlStep->opCode.write(rtctrlProgTriggerReset);

    // restart the program
    pRTCtrlStep = pRTCtrlprogram->programStep(progStep++);
    pRTCtrlStep->opCode.write(rtctrlProgJumpLoc);
    pRTCtrlStep->address.write(0);

    // start the program
    pRTCtrlprogram->mode.write(rtctrlModeRun);
  }
  else
  {
    ROS_WARN_STREAM("Setting framerate for "<<cam_->family.read()<<" not implemented");
    return false;
  }

  ROS_INFO("Framerate set to: %f fps", fps);

  return true;
}



/*



void Features::reconfigure(Config *newconfig)  {

    // get the settings
    SettingsBluecougar settings(cam_);
    CameraSettingsBluecougar & cam_settings = settings.cameraSetting;

}



*//*



//-----------------------------------------------------------------------------
// This function makes heavy use of strings. In real world applications
// this can be avoided if optimal performance is crucial. All properties can be modified
// via strings, but most properties can also be modified with numerical (int / double )
// values, which is much faster, but not as descriptive for a sample application
void modifyPropertyValue( const mvIMPACT::acquire::Property& prop, const std::string& param = "", const std::string& index = "" )
//-----------------------------------------------------------------------------
{
	try
	{
		const std::string name(prop.name());
		int valIndex = 0;
		if( prop.isWriteable() )
		{
			if( param.empty() )
			{
				std::cout << "Enter the new value for '" << name << "': ";
				std::string val;
				std::cin >> val;
				// remove the '\n' from the stream
				std::cin.get();
				if( prop.valCount() > 1 )
				{
					std::cout << "'" << name << "' defines " << prop.valCount() << " values. Enter the index (zero-based) of the value to modify: ";
					std::cin >> valIndex;
					// remove the '\n' from the stream
					std::cin.get();
				}
				prop.writeS( val, valIndex );
			}
			else
			{
				if( !index.empty() )
				{
					valIndex = atoi( index.c_str() );
				}
				prop.writeS( param, valIndex );
			}
		}
		else
		{
			std::cout << "'" << name << "' is read-only, thus can't be modified." << std::endl;
		}
	}
	catch( const mvIMPACT::acquire::ImpactAcquireException& e )
	{
		std::cout << "An exception occurred: " << e.getErrorString() << "(error code: " << e.getErrorCodeAsString() << ")" << std::endl;
	}
}

*/






/** Reconfigure features for already open device.
 *
 *  For each supported feature that has changed, update the device.
 *
 *  @pre feature_set_ initialized
 *  @pre oldconfig_ has previous settings
 *
 *  @param newconfig [in,out] configuration parameters, may be updated
 *         to conform with device restrictions.
 *
 *  @post oldconfig_ settings updated
 */
void Features::reconfigure(Config *newconfig)
{
//  SettingsBlueCOUGAR settings(cam_);
  CameraSettingsBlueDevice  cam_settings(cam_);

  newconfig->x_offset = clamp(newconfig->x_offset, 0, cam_settings.aoiWidth.read(plMaxValue) - 16);
  newconfig->y_offset = clamp(newconfig->y_offset, 0, cam_settings.aoiHeight.read(plMaxValue) - 16);
  configure(cam_settings.aoiStartX, newconfig->x_offset);
  configure(cam_settings.aoiStartY, newconfig->y_offset);
  newconfig->roi_width = clamp(newconfig->roi_width, 16, cam_settings.aoiWidth.read(plMaxValue));
  newconfig->roi_height = clamp(newconfig->roi_height, 16, cam_settings.aoiHeight.read(plMaxValue));
  configure(cam_settings.aoiWidth, newconfig->roi_width);
  configure(cam_settings.aoiHeight, newconfig->roi_height);

  configure(cam_settings.autoControlParameters.desiredAverageGreyValue, newconfig->exposure);

  // shutter control
  if ((newconfig->shutter != oldconfig_.shutter) || (newconfig->shutter_auto != oldconfig_.shutter_auto)
      || (newconfig->shutter_auto_max != oldconfig_.shutter_auto_max) || (newconfig->shutter_auto_min
      != oldconfig_.shutter_auto_min))
  {
    mvIMPACT::acquire::TAutoExposureControl aec = (newconfig->shutter_auto ? aecOn : aecOff);
    cam_settings.autoExposeControl.write(aec);

    //		if(oldconfig_.shutter_auto && !newconfig->shutter_auto){
    //			exposure_time = sToUs(image_info_.exposure_time);
    //			ROS_INFO("auto shutter --> man_shutter, using last auto shutter time: %f ms", image_info_.exposure_time*1000.0);
    //		}
    //		else {
    //			exposure_time = sToUs(newconfig->shutter);
    //		}



    double frame_time = computeFrameTime();
    double exp_max = 1.0 / fps_ - frame_time - 2 * TRIGGER_PULSE_WIDTH;
    exp_max *= 0.9; // leave some safety margin for max. exposure

    newconfig->shutter = clamp(newconfig->shutter, 0.0, exp_max);
    newconfig->shutter_auto_min = clamp(newconfig->shutter_auto_min, double(0), exp_max);
    newconfig->shutter_auto_max = clamp(newconfig->shutter_auto_max, double(0), exp_max);

    ROS_WARN_STREAM_COND(newconfig->shutter == exp_max, "limited shutter time to "<<exp_max*1000<< " ms not to affect framerate");
    ROS_WARN_STREAM_COND(newconfig->shutter_auto_min == exp_max, "limited min. auto shutter time to "<<exp_max*1000<< " ms not to affect framerate");
    ROS_WARN_STREAM_COND(newconfig->shutter_auto_max == exp_max, "limited max. auto shutter time to "<<exp_max*1000<< " ms not to affect framerate");

    int exposure_time = sToUs(newconfig->shutter);
    configure(cam_settings.expose_us, exposure_time, &exposure_time);
    newconfig->shutter = usToS(exposure_time);

    int exposure_limit = sToUs(newconfig->shutter_auto_min);
    configure(cam_settings.autoControlParameters.exposeLowerLimit_us, exposure_limit, &exposure_limit);
    newconfig->shutter_auto_min = usToS(exposure_limit);

    exposure_limit = sToUs(newconfig->shutter_auto_max);
    configure(cam_settings.autoControlParameters.exposeUpperLimit_us, exposure_limit, &exposure_limit);
    newconfig->shutter_auto_max = usToS(exposure_limit);

  }

  // gain control
  mvIMPACT::acquire::TAutoGainControl agc = (newconfig->gain_auto ? agcOn : agcOff);
  configure(cam_settings.autoGainControl, agc, &agc);
  newconfig->gain_auto = (agc == agcOn ? true : false);
  configure(cam_settings.gain_dB, newconfig->gain);
  configure(cam_settings.autoControlParameters.gainLowerLimit_dB, newconfig->gain_auto_min, &newconfig->gain_auto_min);
  configure(cam_settings.autoControlParameters.gainUpperLimit_dB, newconfig->gain_auto_max, &newconfig->gain_auto_max);

  // auto control speed
  try
  {
    cam_settings.autoControlParameters.controllerSpeed.writeS(newconfig->auto_control_speed);
  }
  catch (ImpactAcquireException & e)
  {
    ROS_WARN("couldn't set auto controller speed to %s, reason: %s", newconfig->auto_control_speed.c_str(), e.getErrorString().c_str());
    newconfig->auto_control_speed = oldconfig_.auto_control_speed;
  }

  // framerate
  if (newconfig->frame_rate != oldconfig_.frame_rate)
  {
    if (!setFramerate(newconfig->frame_rate, &newconfig->frame_rate))
    {
      newconfig->frame_rate = oldconfig_.frame_rate;
    }
    else
    {
      fps_ = newconfig->frame_rate;
    }
  }

  setHDR(newconfig->hdr_mode, &newconfig->hdr_mode);

  setColorCoding(newconfig->color_coding, &newconfig->color_coding);

  setPixelClock(newconfig->pixel_clock, &newconfig->pixel_clock);

  if (newconfig->auto_query_values)
  {
    newconfig->gain = image_info_.gain;
    newconfig->shutter = image_info_.exposure_time;
    newconfig->auto_query_values = false;

    // TODO: handle 16 bit
    if(image_info_.color_coding.find("bayer")!=std::string::npos)
      newconfig->color_coding = "raw8";
    else
      newconfig->color_coding = image_info_.color_coding;
  }

  // save modified values
  oldconfig_ = *newconfig;
}

bool Features::setHDR(const std::string & hdr_suggested, std::string * hdr_returned)
{
  HDRControl * hdrc;

  std::string retval = oldconfig_.hdr_mode;

  CameraSettingsBlueFOX bluefox_settings(cam_);
  HDRControl & bluefox_hdrc = bluefox_settings.getHDRControl();
  CameraSettingsBlueCOUGAR bluecougar_settings(cam_);
  HDRControl & bluecougar_hdrc = bluecougar_settings.getHDRControl();

  if (cam_->family.read() == "mvBlueFOX")
  {
    hdrc = &bluefox_hdrc;
  }
  else if (cam_->family.read() == "mvBlueCOUGAR")
  {
    hdrc = &bluecougar_hdrc;
  }
  else {
      return false;
  }

  try
  {
    if (hdrc->HDREnable.isValid())
    {
      if (hdr_suggested == matrix_vision_camera::MatrixVisionCamera_hdr_off)
      {
        //
        hdrc->HDREnable.write(bFalse, 0);
        retval = hdr_suggested;
      }
      else if (hdr_suggested == matrix_vision_camera::MatrixVisionCamera_hdr_user)
      {
        retval = oldconfig_.hdr_mode;
      }
      else
      {
        hdrc->HDREnable.write(bTrue);
        hdrc->HDRMode.writeS(hdr_suggested);
        retval = hdr_suggested;
      }
    }
    else
    {
      retval = matrix_vision_camera::MatrixVisionCamera_hdr_off;
    }
  }
  catch (ImpactAcquireException & e)
  {
    ROS_WARN_STREAM(
        "Unable to set HDR mode to: "<< hdr_suggested << " reason: "<<e.getErrorCodeAsString() << " EString:" << e.getErrorString());
    if (hdr_returned)
      *hdr_returned = retval;
    return false;
  }

  if (hdr_returned)
    *hdr_returned = retval;

  return true;
}

bool Features::setColorCoding(const std::string & cc_suggested, std::string * cc_returned)
{
  // set color mode
  mvIMPACT::acquire::ImageDestination img_dest(cam_);
  bool success = false;

  try
  {
    for (unsigned int i = 0; i < img_dest.pixelFormat.dictSize(); i++)
    {
      if (matrix_vision_camera::pixelFormat(cc_suggested) == img_dest.pixelFormat.getTranslationDictValue(i))
      {
        img_dest.pixelFormat.write(matrix_vision_camera::pixelFormat(cc_suggested));
        success = true;
        break;
      }
    }

    if (!success)
    {
      img_dest.pixelFormat.write(mvIMPACT::acquire::idpfAuto);
    }

    if (cc_returned)
      *cc_returned = matrix_vision_camera::pixelFormat(img_dest.pixelFormat.read());

    return success;
  }
  catch (ImpactAcquireException & e)
  {
    ROS_WARN_STREAM("Unable to set color coding to: "<< cc_suggested << " reason: "<<e.getErrorCodeAsString());
    if (cc_returned)
      *cc_returned = oldconfig_.color_coding;
    return false;
  }

  return success;
}

bool Features::setPixelClock(const double & px_clock_suggested, double * px_clock_returned)
{
  CameraSettingsBlueDevice settings(cam_);
  PropertyICameraPixelClock & pixel_clock = settings.pixelClock_KHz;
  int pixel_clock_kHz = MHzToKHz(px_clock_suggested);
  TCameraPixelClock closest_pixel_clock = cpcStandard;

  double retval = oldconfig_.pixel_clock;
  bool success = false;

  for (unsigned int i = 0; i < pixel_clock.dictSize(); i++)
  {
    const TCameraPixelClock & px_clk = pixel_clock.getTranslationDictValue(i);
    int diff1 = abs(px_clk - pixel_clock_kHz);
    int diff2 = abs(px_clk - closest_pixel_clock);
    if (diff1 < diff2)
    {
      closest_pixel_clock = px_clk;
    }
  }

  if (closest_pixel_clock != 0)
  {
    pixel_clock.write(closest_pixel_clock);
    retval = kHzToMHz(closest_pixel_clock);
    success = true;
  }
  else
  {
    retval = kHzToMHz(pixel_clock.read());
    ROS_WARN("unable to find desired pixel clock");
    success = false;
  }

  if (px_clock_returned)
    *px_clock_returned = retval;

  return success;
}
