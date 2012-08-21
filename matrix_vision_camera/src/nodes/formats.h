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

#ifndef FORMATS_H_
#define FORMATS_H_

#include <string>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

namespace matrix_vision_camera{

std::string pixelFormat(const mvIMPACT::acquire::TImageBufferPixelFormat &format);

std::string pixelFormat(const mvIMPACT::acquire::TImageDestinationPixelFormat &format);

mvIMPACT::acquire::TImageDestinationPixelFormat pixelFormat(const std::string &format);

std::string bayerString(const mvIMPACT::acquire::TBayerMosaicParity &pattern, unsigned int bits);

}; // end namespace matrix_vision_camera

#endif /* FORMATS_H_ */
