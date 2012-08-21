/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012, Markus Achtelik
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "formats.h"
#include <sensor_msgs/image_encodings.h>
#include <map>

namespace matrix_vision_camera{

std::string pixelFormat(const mvIMPACT::acquire::TImageBufferPixelFormat &format)
{
  switch (format)
  {
    case mvIMPACT::acquire::ibpfMono8:
      return sensor_msgs::image_encodings::MONO8;
    case mvIMPACT::acquire::ibpfMono16:
      return sensor_msgs::image_encodings::MONO16;
    case mvIMPACT::acquire::ibpfRGB888Packed:
      return sensor_msgs::image_encodings::BGR8;
    case mvIMPACT::acquire::ibpfRGB161616Packed:
      return sensor_msgs::image_encodings::BGR16;
    case mvIMPACT::acquire::ibpfRGBx888Packed:
      return sensor_msgs::image_encodings::BGRA8;
    case mvIMPACT::acquire::ibpfAuto:
      return "auto";
    default:
      return "";
  }
}

std::string pixelFormat(const mvIMPACT::acquire::TImageDestinationPixelFormat &format)
{
  switch (format)
  {
    case mvIMPACT::acquire::idpfMono8:
      return sensor_msgs::image_encodings::MONO8;
    case mvIMPACT::acquire::idpfMono16:
      return sensor_msgs::image_encodings::MONO16;
    case mvIMPACT::acquire::idpfRGB888Packed:
      return sensor_msgs::image_encodings::BGR8;
    case mvIMPACT::acquire::idpfRGB161616Packed:
      return sensor_msgs::image_encodings::BGR16;
    case mvIMPACT::acquire::idpfRGBx888Packed:
      return sensor_msgs::image_encodings::BGRA8;
    case mvIMPACT::acquire::idpfRaw:
      return "raw8";
    case mvIMPACT::acquire::idpfAuto:
      return "auto";
    default:
      return "";
  }
}

mvIMPACT::acquire::TImageDestinationPixelFormat pixelFormat(const std::string &format)
{
  if(format==sensor_msgs::image_encodings::MONO8)
    return mvIMPACT::acquire::idpfMono8;
  if(format==sensor_msgs::image_encodings::MONO16)
    return mvIMPACT::acquire::idpfMono16;
  if(format==sensor_msgs::image_encodings::BGR8)
    return mvIMPACT::acquire::idpfRGB888Packed;
  if(format==sensor_msgs::image_encodings::BGR16)
    return mvIMPACT::acquire::idpfRGB161616Packed;
  if(format==sensor_msgs::image_encodings::BGRA8)
    return mvIMPACT::acquire::idpfRGBx888Packed;
  if(format == "raw8")
    return mvIMPACT::acquire::idpfRaw;

  return mvIMPACT::acquire::idpfAuto;
}

std::string bayerString(const mvIMPACT::acquire::TBayerMosaicParity &pattern, unsigned int bits)
{
  if (bits == 8)
  {
    switch (pattern)
    {
      case mvIMPACT::acquire::bmpRG:
        return sensor_msgs::image_encodings::BAYER_RGGB8;
      case mvIMPACT::acquire::bmpGB:
        return sensor_msgs::image_encodings::BAYER_GBRG8;
      case mvIMPACT::acquire::bmpGR:
        return sensor_msgs::image_encodings::BAYER_GRBG8;
      case mvIMPACT::acquire::bmpBG:
        return sensor_msgs::image_encodings::BAYER_BGGR8;
      default:
        return sensor_msgs::image_encodings::MONO8;
    }
  }
  else if (bits == 16)
  {
    switch (pattern)
    {
      case mvIMPACT::acquire::bmpRG:
        return sensor_msgs::image_encodings::BAYER_RGGB16;
      case mvIMPACT::acquire::bmpGB:
        return sensor_msgs::image_encodings::BAYER_GBRG16;
      case mvIMPACT::acquire::bmpGR:
        return sensor_msgs::image_encodings::BAYER_GRBG16;
      case mvIMPACT::acquire::bmpBG:
        return sensor_msgs::image_encodings::BAYER_BGGR16;
      default:
        return sensor_msgs::image_encodings::MONO16;
    }
  }

  return "";
}

}; // end namespace matrix_vision_camera
