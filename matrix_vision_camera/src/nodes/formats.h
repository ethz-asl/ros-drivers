/*
 * formats.h
 *
 *  Created on: Dec 15, 2011
 *      Author: acmarkus
 */

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
