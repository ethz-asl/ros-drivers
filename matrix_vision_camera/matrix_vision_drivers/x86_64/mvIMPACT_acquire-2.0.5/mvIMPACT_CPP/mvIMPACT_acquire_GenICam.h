//-----------------------------------------------------------------------------
#ifndef mvIMPACT_acquire_GenICam_CPP_autogen_h
#if !defined(DOXYGEN_SHOULD_SKIP_THIS) && !defined(WRAP_ANY)
#	define mvIMPACT_acquire_GenICam_CPP_autogen_h mvIMPACT_acquire_GenICam_CPP_autogen_h
#endif // DOXYGEN_SHOULD_SKIP_THIS && WRAP_ANY
//-----------------------------------------------------------------------------
// AUTOMATICALLY GENERATED CODE. DO NOT EDIT!!!

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#ifdef SWIG
#	ifdef SWIGPYTHON
#		define WRAP_PYTHON
#	endif
#	ifdef SWIGJAVA
#		define WRAP_JAVA
#	endif
#endif

#ifdef WRAP_PYTHON
#	define PYTHON_ONLY(X) X
#	define PYTHON_CPP_SWITCH(PYTHON_WRAPPER_CODE,CPP_WRAPPER_CODE) PYTHON_WRAPPER_CODE
#	ifndef WRAP_ANY
#		define WRAP_ANY
#	endif // #ifndef WRAP_ANY
#else // #ifdef WRAP_PYTHON
#	define PYTHON_ONLY(X)
#	define PYTHON_CPP_SWITCH(PYTHON_WRAPPER_CODE,CPP_WRAPPER_CODE) CPP_WRAPPER_CODE
#endif // #ifdef WRAP_PYTHON

#ifdef WRAP_DOTNET
#	define DOTNET_ONLY(X) X
#	define DOTNET_CPP_SWITCH(DOTNET_WRAPPER_CODE,CPP_WRAPPER_CODE) DOTNET_WRAPPER_CODE
#	ifndef WRAP_ANY
#		define WRAP_ANY
#	endif // #ifndef WRAP_ANY
#else // #ifdef WRAP_DOTNET
#	define DOTNET_ONLY(X)
#	define DOTNET_CPP_SWITCH(DOTNET_WRAPPER_CODE,CPP_WRAPPER_CODE) CPP_WRAPPER_CODE
#endif // #ifdef WRAP_DOTNET

#ifdef _MSC_VER // is Microsoft compiler?
#	pragma warning( push )
#	if _MSC_VER < 1300 // is 'old' VC 6 compiler?
#		pragma warning( disable : 4786 ) // 'identifier was truncated to '255' characters in the debug information'
#		define __FUNCTION__ "No function name information as the __FUNCTION__ macro is not supported by this(VC 6) compiler"
#		pragma message( "WARNING: This header(" __FILE__ ") uses the __FUNCTION__ macro, which is not supported by this compiler. A default definition(\"" __FUNCTION__ "\") will be used!" )
#		pragma message( "WARNING: This header(" __FILE__ ") uses inheritance for exception classes. However this compiler can't handle this correctly. Trying to catch a specific exception by writing a catch block for a base class will not work!" )
#	endif // #if _MSC_VER < 1300
#	if _MSC_VER >= 1400 // is at least VC 2005 compiler?
#		include <assert.h>
#	endif // #if _MSC_VER >= 1400
#	pragma warning( disable : 4512 ) // 'assignment operator could not be generated' (reason: assignment operators declared 'private' but not implemented)
#endif // #ifdef _MSC_VER

namespace mvIMPACT {
	namespace acquire {
		namespace GenICam {

/// \defgroup GenICamInterface GenICam interface layout
/// This group contains classes that will be available if the device is used
/// with the <b>mvIMPACT::acquire::dilGenICam</b> interface layout.
///
/// \ingroup CommonInterface
///
/// @{

//-----------------------------------------------------------------------------
/// \brief Category for Device information and control.
///
/// Category for Device information and control.
class DeviceControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::DeviceControl</b> object.
	explicit DeviceControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			deviceVendorName(),
			deviceModelName(),
			deviceManufacturerInfo(),
			deviceVersion(),
			deviceFirmwareVersion(),
			deviceSFNCVersionMajor(),
			deviceSFNCVersionMinor(),
			deviceSFNCVersionSubMinor(),
			deviceManifestEntrySelector(),
			deviceManifestXMLMajorVersion(),
			deviceManifestXMLMinorVersion(),
			deviceManifestXMLSubMinorVersion(),
			deviceManifestSchemaMajorVersion(),
			deviceManifestSchemaMinorVersion(),
			deviceManifestPrimaryURL(),
			deviceManifestSecondaryURL(),
			deviceID(),
			deviceUserID(),
			deviceReset(),
			deviceRegistersStreamingStart(),
			deviceRegistersStreamingEnd(),
			deviceRegistersCheck(),
			deviceRegistersValid(),
			deviceMaxThroughput(),
			deviceTemperatureSelector(),
			deviceTemperature(),
			deviceClockSelector(),
			deviceClockFrequency(),
			deviceSerialPortSelector(),
			deviceSerialPortBaudRate(),
			deviceScanType(),
			timestamp(),
			timestampReset(),
			mvDeviceTemperatureUpperLimit(),
			mvDeviceTemperatureLowerLimit(),
			mvDeviceTemperatureLimitHysteresis(),
			mvDeviceClockFrequency(),
			mvDeviceClockGranularity(),
			mvDeviceSensorName(),
			mvDeviceSensorColorMode(),
			mvDeviceFPGAVersion(),
			mvDeviceFirmwareSource()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( deviceVendorName, "DeviceVendorName" );
		locator.bindComponent( deviceModelName, "DeviceModelName" );
		locator.bindComponent( deviceManufacturerInfo, "DeviceManufacturerInfo" );
		locator.bindComponent( deviceVersion, "DeviceVersion" );
		locator.bindComponent( deviceFirmwareVersion, "DeviceFirmwareVersion" );
		locator.bindComponent( deviceSFNCVersionMajor, "DeviceSFNCVersionMajor" );
		locator.bindComponent( deviceSFNCVersionMinor, "DeviceSFNCVersionMinor" );
		locator.bindComponent( deviceSFNCVersionSubMinor, "DeviceSFNCVersionSubMinor" );
		locator.bindComponent( deviceManifestEntrySelector, "DeviceManifestEntrySelector" );
		locator.bindComponent( deviceManifestXMLMajorVersion, "DeviceManifestXMLMajorVersion" );
		locator.bindComponent( deviceManifestXMLMinorVersion, "DeviceManifestXMLMinorVersion" );
		locator.bindComponent( deviceManifestXMLSubMinorVersion, "DeviceManifestXMLSubMinorVersion" );
		locator.bindComponent( deviceManifestSchemaMajorVersion, "DeviceManifestSchemaMajorVersion" );
		locator.bindComponent( deviceManifestSchemaMinorVersion, "DeviceManifestSchemaMinorVersion" );
		locator.bindComponent( deviceManifestPrimaryURL, "DeviceManifestPrimaryURL" );
		locator.bindComponent( deviceManifestSecondaryURL, "DeviceManifestSecondaryURL" );
		locator.bindComponent( deviceID, "DeviceID" );
		locator.bindComponent( deviceUserID, "DeviceUserID" );
		locator.bindComponent( deviceReset, "DeviceReset@i" );
		locator.bindComponent( deviceRegistersStreamingStart, "DeviceRegistersStreamingStart@i" );
		locator.bindComponent( deviceRegistersStreamingEnd, "DeviceRegistersStreamingEnd@i" );
		locator.bindComponent( deviceRegistersCheck, "DeviceRegistersCheck@i" );
		locator.bindComponent( deviceRegistersValid, "DeviceRegistersValid" );
		locator.bindComponent( deviceMaxThroughput, "DeviceMaxThroughput" );
		locator.bindComponent( deviceTemperatureSelector, "DeviceTemperatureSelector" );
		locator.bindComponent( deviceTemperature, "DeviceTemperature" );
		locator.bindComponent( deviceClockSelector, "DeviceClockSelector" );
		locator.bindComponent( deviceClockFrequency, "DeviceClockFrequency" );
		locator.bindComponent( deviceSerialPortSelector, "DeviceSerialPortSelector" );
		locator.bindComponent( deviceSerialPortBaudRate, "DeviceSerialPortBaudRate" );
		locator.bindComponent( deviceScanType, "DeviceScanType" );
		locator.bindComponent( timestamp, "Timestamp" );
		locator.bindComponent( timestampReset, "TimestampReset@i" );
		locator.bindComponent( mvDeviceTemperatureUpperLimit, "mvDeviceTemperatureUpperLimit" );
		locator.bindComponent( mvDeviceTemperatureLowerLimit, "mvDeviceTemperatureLowerLimit" );
		locator.bindComponent( mvDeviceTemperatureLimitHysteresis, "mvDeviceTemperatureLimitHysteresis" );
		locator.bindComponent( mvDeviceClockFrequency, "mvDeviceClockFrequency" );
		locator.bindComponent( mvDeviceClockGranularity, "mvDeviceClockGranularity" );
		locator.bindComponent( mvDeviceSensorName, "mvDeviceSensorName" );
		locator.bindComponent( mvDeviceSensorColorMode, "mvDeviceSensorColorMode" );
		locator.bindComponent( mvDeviceFPGAVersion, "mvDeviceFPGAVersion" );
		locator.bindComponent( mvDeviceFirmwareSource, "mvDeviceFirmwareSource" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Name of the manufacturer of the device.
	///
	/// Name of the manufacturer of the device.
	PropertyS deviceVendorName;
	/// \brief Model of the device.
	///
	/// Model of the device.
	PropertyS deviceModelName;
	/// \brief Manufacturer information about the device.
	///
	/// Manufacturer information about the device.
	PropertyS deviceManufacturerInfo;
	/// \brief Version of the device.
	///
	/// Version of the device.
	PropertyS deviceVersion;
	/// \brief Version of the firmware in the device.
	///
	/// Version of the firmware in the device.
	PropertyS deviceFirmwareVersion;
	/// \brief Major Version of the Standard Features Naming Convention that was used to create the device`s XML.
	///
	/// Major Version of the Standard Features Naming Convention that was used to create the device`s XML.
	PropertyI64 deviceSFNCVersionMajor;
	/// \brief Minor Version of the Standard Features Naming Convention that was used to create the device`s XML.
	///
	/// Minor Version of the Standard Features Naming Convention that was used to create the device`s XML.
	PropertyI64 deviceSFNCVersionMinor;
	/// \brief Sub Minor Version of Standard Features Naming Convention that was used to create the device`s XML.
	///
	/// Sub Minor Version of Standard Features Naming Convention that was used to create the device`s XML.
	PropertyI64 deviceSFNCVersionSubMinor;
	/// \brief Selects the manifest entry to reference.
	///
	/// Selects the manifest entry to reference.
	PropertyI64 deviceManifestEntrySelector;
	/// \brief Indicates the major version number of the XML file of the selected manifest entry.
	///
	/// Indicates the major version number of the XML file of the selected manifest entry.
	PropertyI64 deviceManifestXMLMajorVersion;
	/// \brief Indicates the minor version number of the XML file of the selected manifest entry.
	///
	/// Indicates the minor version number of the XML file of the selected manifest entry.
	PropertyI64 deviceManifestXMLMinorVersion;
	/// \brief Indicates the subminor version number of the XML file of the selected manifest entry.
	///
	/// Indicates the subminor version number of the XML file of the selected manifest entry.
	PropertyI64 deviceManifestXMLSubMinorVersion;
	/// \brief Indicates the major version number of the schema file of the selected manifest entry.
	///
	/// Indicates the major version number of the schema file of the selected manifest entry.
	PropertyI64 deviceManifestSchemaMajorVersion;
	/// \brief Indicates the minor version number of the schema file of the selected manifest entry.
	///
	/// Indicates the minor version number of the schema file of the selected manifest entry.
	PropertyI64 deviceManifestSchemaMinorVersion;
	/// \brief Indicates the first URL to the XML device description file of the selected manifest entry.
	///
	/// Indicates the first URL to the XML device description file of the selected manifest entry.
	PropertyS deviceManifestPrimaryURL;
	/// \brief Indicates the second URL to the XML device description file of the selected manifest entry.
	///
	/// Indicates the second URL to the XML device description file of the selected manifest entry.
	PropertyS deviceManifestSecondaryURL;
	/// \brief Device Identifier (serial number).
	///
	/// Device Identifier (serial number).
	PropertyS deviceID;
	/// \brief User-programmable Device Identifier.
	///
	/// User-programmable Device Identifier.
	PropertyS deviceUserID;
	/// \brief Resets the device to its power up state.
	///
	/// Resets the device to its power up state.
	Method deviceReset;
	/// \brief Prepare the device for registers streaming without checking for consistency.
	///
	/// Prepare the device for registers streaming without checking for consistency.
	Method deviceRegistersStreamingStart;
	/// \brief Announce the end of registers streaming.
	///
	/// Announce the end of registers streaming. This will do a register set validation for consistency and activate it. This will also update the DeviceRegistersValid flag.
	Method deviceRegistersStreamingEnd;
	/// \brief Perform the validation of the current register set for consistency.
	///
	/// Perform the validation of the current register set for consistency. This will update the DeviceRegistersValid flag.
	Method deviceRegistersCheck;
	/// \brief Returns if the current register set is valid and consistent.
	///
	/// Returns if the current register set is valid and consistent.
	PropertyIBoolean deviceRegistersValid;
	/// \brief Maximum bandwidth of the data that can be streamed out of the device.
	///
	/// Maximum bandwidth of the data that can be streamed out of the device. This can be used to estimate if the network connection can sustain transfer of free-running images from the camera at its maximum speed.
	PropertyI64 deviceMaxThroughput;
	/// \brief Selects the location within the device, where the temperature will be measured.
	///
	/// Selects the location within the device, where the temperature will be measured.
	PropertyI64 deviceTemperatureSelector;
	/// \brief Device temperature in degrees Celsius (C).
	///
	/// Device temperature in degrees Celsius (C). It is measured at the location selected by DeviceTemperatureSelector.
	PropertyF deviceTemperature;
	/// \brief Selects the clock frequency to access from the device.
	///
	/// Selects the clock frequency to access from the device.
	PropertyI64 deviceClockSelector;
	/// \brief Returns the frequency in Hertz of the selected Clock.
	///
	/// Returns the frequency in Hertz of the selected Clock.
	PropertyF deviceClockFrequency;
	/// \brief Selects which device serial port to control.
	///
	/// Selects which device serial port to control.
	PropertyI64 deviceSerialPortSelector;
	/// \brief This feature controls the baud rate used by the selected serial port.
	///
	/// This feature controls the baud rate used by the selected serial port. Typical values listed should be used whenever possible. Arbitrary values can also be used by defining new enumeration entries.
	PropertyI64 deviceSerialPortBaudRate;
	/// \brief Scan type of the sensor of the device.
	///
	/// Scan type of the sensor of the device.
	PropertyI64 deviceScanType;
	/// \brief Reports the current value of the device timestamp counter.
	///
	/// Reports the current value of the device timestamp counter.
	PropertyI64 timestamp;
	/// \brief Resets the current value of the device timestamp counter.
	///
	/// Resets the current value of the device timestamp counter.
	Method timestampReset;
	/// \brief Upper limit in degrees Celsius(C) for the TemperatureOutOfRange signal.
	///
	/// Upper limit in degrees Celsius(C) for the TemperatureOutOfRange signal.
	PropertyI64 mvDeviceTemperatureUpperLimit;
	/// \brief Lower limit in degrees Celsius(C) for the TemperatureOutOfRange signal.
	///
	/// Lower limit in degrees Celsius(C) for the TemperatureOutOfRange signal.
	PropertyI64 mvDeviceTemperatureLowerLimit;
	/// \brief Hysteresis in degrees Celsius(C) for temperature limits.
	///
	/// Hysteresis in degrees Celsius(C) for temperature limits.
	PropertyI64 mvDeviceTemperatureLimitHysteresis;
	/// \brief Clock frequency of the image sensor of the camera.
	///
	/// Clock frequency of the image sensor of the camera.
	PropertyI64 mvDeviceClockFrequency;
	PropertyI64 mvDeviceClockGranularity;
	/// \brief Shows the name of the sensor.
	///
	/// Shows the name of the sensor.
	PropertyS mvDeviceSensorName;
	/// \brief Shows color mode of the sensor.
	///
	/// Shows color mode of the sensor.
	PropertyI64 mvDeviceSensorColorMode;
	/// \brief Shows version number of the FPGA.
	///
	/// Shows version number of the FPGA.
	PropertyS mvDeviceFPGAVersion;
	/// \brief Shows the location from where the firmware was loaded.
	///
	/// Shows the location from where the firmware was loaded.
	PropertyI64 mvDeviceFirmwareSource;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyS getDeviceVendorName( void ) const { return deviceVendorName; }
	PropertyS getDeviceModelName( void ) const { return deviceModelName; }
	PropertyS getDeviceManufacturerInfo( void ) const { return deviceManufacturerInfo; }
	PropertyS getDeviceVersion( void ) const { return deviceVersion; }
	PropertyS getDeviceFirmwareVersion( void ) const { return deviceFirmwareVersion; }
	PropertyI64 getDeviceSFNCVersionMajor( void ) const { return deviceSFNCVersionMajor; }
	PropertyI64 getDeviceSFNCVersionMinor( void ) const { return deviceSFNCVersionMinor; }
	PropertyI64 getDeviceSFNCVersionSubMinor( void ) const { return deviceSFNCVersionSubMinor; }
	PropertyI64 getDeviceManifestEntrySelector( void ) const { return deviceManifestEntrySelector; }
	PropertyI64 getDeviceManifestXMLMajorVersion( void ) const { return deviceManifestXMLMajorVersion; }
	PropertyI64 getDeviceManifestXMLMinorVersion( void ) const { return deviceManifestXMLMinorVersion; }
	PropertyI64 getDeviceManifestXMLSubMinorVersion( void ) const { return deviceManifestXMLSubMinorVersion; }
	PropertyI64 getDeviceManifestSchemaMajorVersion( void ) const { return deviceManifestSchemaMajorVersion; }
	PropertyI64 getDeviceManifestSchemaMinorVersion( void ) const { return deviceManifestSchemaMinorVersion; }
	PropertyS getDeviceManifestPrimaryURL( void ) const { return deviceManifestPrimaryURL; }
	PropertyS getDeviceManifestSecondaryURL( void ) const { return deviceManifestSecondaryURL; }
	PropertyS getDeviceID( void ) const { return deviceID; }
	PropertyS getDeviceUserID( void ) const { return deviceUserID; }
	Method getDeviceReset( void ) const { return deviceReset; }
	Method getDeviceRegistersStreamingStart( void ) const { return deviceRegistersStreamingStart; }
	Method getDeviceRegistersStreamingEnd( void ) const { return deviceRegistersStreamingEnd; }
	Method getDeviceRegistersCheck( void ) const { return deviceRegistersCheck; }
	PropertyIBoolean getDeviceRegistersValid( void ) const { return deviceRegistersValid; }
	PropertyI64 getDeviceMaxThroughput( void ) const { return deviceMaxThroughput; }
	PropertyI64 getDeviceTemperatureSelector( void ) const { return deviceTemperatureSelector; }
	PropertyF getDeviceTemperature( void ) const { return deviceTemperature; }
	PropertyI64 getDeviceClockSelector( void ) const { return deviceClockSelector; }
	PropertyF getDeviceClockFrequency( void ) const { return deviceClockFrequency; }
	PropertyI64 getDeviceSerialPortSelector( void ) const { return deviceSerialPortSelector; }
	PropertyI64 getDeviceSerialPortBaudRate( void ) const { return deviceSerialPortBaudRate; }
	PropertyI64 getDeviceScanType( void ) const { return deviceScanType; }
	PropertyI64 getTimestamp( void ) const { return timestamp; }
	Method getTimestampReset( void ) const { return timestampReset; }
	PropertyI64 getmvDeviceTemperatureUpperLimit( void ) const { return mvDeviceTemperatureUpperLimit; }
	PropertyI64 getmvDeviceTemperatureLowerLimit( void ) const { return mvDeviceTemperatureLowerLimit; }
	PropertyI64 getmvDeviceTemperatureLimitHysteresis( void ) const { return mvDeviceTemperatureLimitHysteresis; }
	PropertyI64 getmvDeviceClockFrequency( void ) const { return mvDeviceClockFrequency; }
	PropertyI64 getmvDeviceClockGranularity( void ) const { return mvDeviceClockGranularity; }
	PropertyS getmvDeviceSensorName( void ) const { return mvDeviceSensorName; }
	PropertyI64 getmvDeviceSensorColorMode( void ) const { return mvDeviceSensorColorMode; }
	PropertyS getmvDeviceFPGAVersion( void ) const { return mvDeviceFPGAVersion; }
	PropertyI64 getmvDeviceFirmwareSource( void ) const { return mvDeviceFirmwareSource; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category for Image Format Control features.
///
/// Category for Image Format Control features.
class ImageFormatControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::ImageFormatControl</b> object.
	explicit ImageFormatControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			sensorWidth(),
			sensorHeight(),
			sensorTaps(),
			sensorDigitizationTaps(),
			widthMax(),
			heightMax(),
			width(),
			height(),
			offsetX(),
			offsetY(),
			mvSensorLineOffsetSelector(),
			mvSensorLineOffset(),
			mvSensorLinePeriod(),
			linePitch(),
			binningHorizontal(),
			binningVertical(),
			decimationHorizontal(),
			decimationVertical(),
			reverseX(),
			reverseY(),
			pixelFormat(),
			pixelCoding(),
			pixelSize(),
			pixelColorFilter(),
			pixelDynamicRangeMin(),
			pixelDynamicRangeMax(),
			testImageSelector()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( sensorWidth, "SensorWidth" );
		locator.bindComponent( sensorHeight, "SensorHeight" );
		locator.bindComponent( sensorTaps, "SensorTaps" );
		locator.bindComponent( sensorDigitizationTaps, "SensorDigitizationTaps" );
		locator.bindComponent( widthMax, "WidthMax" );
		locator.bindComponent( heightMax, "HeightMax" );
		locator.bindComponent( width, "Width" );
		locator.bindComponent( height, "Height" );
		locator.bindComponent( offsetX, "OffsetX" );
		locator.bindComponent( offsetY, "OffsetY" );
		locator.bindComponent( mvSensorLineOffsetSelector, "mvSensorLineOffsetSelector" );
		locator.bindComponent( mvSensorLineOffset, "mvSensorLineOffset" );
		locator.bindComponent( mvSensorLinePeriod, "mvSensorLinePeriod" );
		locator.bindComponent( linePitch, "LinePitch" );
		locator.bindComponent( binningHorizontal, "BinningHorizontal" );
		locator.bindComponent( binningVertical, "BinningVertical" );
		locator.bindComponent( decimationHorizontal, "DecimationHorizontal" );
		locator.bindComponent( decimationVertical, "DecimationVertical" );
		locator.bindComponent( reverseX, "ReverseX" );
		locator.bindComponent( reverseY, "ReverseY" );
		locator.bindComponent( pixelFormat, "PixelFormat" );
		locator.bindComponent( pixelCoding, "PixelCoding" );
		locator.bindComponent( pixelSize, "PixelSize" );
		locator.bindComponent( pixelColorFilter, "PixelColorFilter" );
		locator.bindComponent( pixelDynamicRangeMin, "PixelDynamicRangeMin" );
		locator.bindComponent( pixelDynamicRangeMax, "PixelDynamicRangeMax" );
		locator.bindComponent( testImageSelector, "TestImageSelector" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Effective width of the sensor in pixels.
	///
	/// Effective width of the sensor in pixels.
	PropertyI64 sensorWidth;
	/// \brief Effective height of the sensor in pixels.
	///
	/// Effective height of the sensor in pixels.
	PropertyI64 sensorHeight;
	/// \brief Number of taps of the camera sensor.
	///
	/// Number of taps of the camera sensor.
	PropertyI64 sensorTaps;
	/// \brief Number of digitized samples outputted simultaneously by the camera A/D conversion stage.
	///
	/// Number of digitized samples outputted simultaneously by the camera A/D conversion stage.
	PropertyI64 sensorDigitizationTaps;
	/// \brief Maximum width (in pixels) of the image.
	///
	/// Maximum width (in pixels) of the image. The dimension is calculated after horizontal binning, decimation or any other function changing the horizontal dimension of the image.
	PropertyI64 widthMax;
	/// \brief Maximum height (in pixels) of the image.
	///
	/// Maximum height (in pixels) of the image. This dimension is calculated after vertical binning, decimation or any other function changing the vertical dimension of the image
	PropertyI64 heightMax;
	/// \brief Width of the image provided by the device (in pixels).
	///
	/// Width of the image provided by the device (in pixels).
	PropertyI64 width;
	/// \brief Height of the image provided by the device (in pixels).
	///
	/// Height of the image provided by the device (in pixels).
	PropertyI64 height;
	/// \brief Horizontal offset from the origin to the region of interest (in pixels).
	///
	/// Horizontal offset from the origin to the region of interest (in pixels).
	PropertyI64 offsetX;
	/// \brief Vertical offset from the origin to the region of interest (in pixels).
	///
	/// Vertical offset from the origin to the region of interest (in pixels).
	PropertyI64 offsetY;
	/// \brief Selects the sensor to configure
	///
	/// Selects the sensor to configure
	PropertyI64 mvSensorLineOffsetSelector;
	/// \brief Sets the offset of the sensor selected by mvSensorLineOffsetSelector.
	///
	/// Sets the offset of the sensor selected by mvSensorLineOffsetSelector.
	PropertyI64 mvSensorLineOffset;
	/// \brief Time in nanoseconds for one line
	///
	/// Time in nanoseconds for one line
	PropertyI64 mvSensorLinePeriod;
	/// \brief Total number of bytes between 2 successive lines.
	///
	/// Total number of bytes between 2 successive lines. This feature is used to facilitate alignment of image data.
	PropertyI64 linePitch;
	/// \brief Number of horizontal photo-sensitive cells to combine together.
	///
	/// Number of horizontal photo-sensitive cells to combine together. This increases the intensity (or signal to noise ratio) of the pixels and reduces the horizontal resolution (width) of the image.
	PropertyI64 binningHorizontal;
	/// \brief Number of vertical photo-sensitive cells to combine together.
	///
	/// Number of vertical photo-sensitive cells to combine together. This increases the intensity (or signal to noise ratio) of the pixels and reduces the vertical resolution (height) of the image.
	PropertyI64 binningVertical;
	/// \brief Horizontal sub-sampling of the image.
	///
	/// Horizontal sub-sampling of the image. This reduces the horizontal resolution (width) of the image by the specified horizontal decimation factor.
	PropertyI64 decimationHorizontal;
	/// \brief Vertical sub-sampling of the image.
	///
	/// Vertical sub-sampling of the image. This reduces the vertical resolution (height) of the image by the specified vertical decimation factor.
	PropertyI64 decimationVertical;
	/// \brief Flip horizontally the image sent by the device.
	///
	/// Flip horizontally the image sent by the device. The ROI is applied after the flipping.
	PropertyIBoolean reverseX;
	/// \brief Flip vertically the image sent by the device.
	///
	/// Flip vertically the image sent by the device. The ROI is applied after the flipping.
	PropertyIBoolean reverseY;
	/// \brief Format of the pixel provided by the device.
	///
	/// Format of the pixel provided by the device. It represents all the information provided by PixelCoding, PixelSize, PixelColorFilter but combined in a single value.
	PropertyI64 pixelFormat;
	/// \brief Coding of the pixels in the image.
	///
	/// Coding of the pixels in the image. Raw gives the data in the native format of the sensor.
	PropertyI64 pixelCoding;
	/// \brief Total size in bits of a pixel of the image.
	///
	/// Total size in bits of a pixel of the image.
	PropertyI64 pixelSize;
	/// \brief Type of color filter that is applied to the image.
	///
	/// Type of color filter that is applied to the image.
	PropertyI64 pixelColorFilter;
	/// \brief Minimum value that can be returned during the digitization process.
	///
	/// Minimum value that can be returned during the digitization process. This corresponds to the darkest value of the camera. For color camera, this returns the smallest value that each color component can take.
	PropertyI64 pixelDynamicRangeMin;
	/// \brief Maximum value that will be returned during the digitization process.
	///
	/// Maximum value that will be returned during the digitization process. This corresponds to the brightest value of the camera. For color camera, this returns the biggest value that each color component can take.
	PropertyI64 pixelDynamicRangeMax;
	/// \brief Selects the type of test image that is sent by the device.
	///
	/// Selects the type of test image that is sent by the device.
	PropertyI64 testImageSelector;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getSensorWidth( void ) const { return sensorWidth; }
	PropertyI64 getSensorHeight( void ) const { return sensorHeight; }
	PropertyI64 getSensorTaps( void ) const { return sensorTaps; }
	PropertyI64 getSensorDigitizationTaps( void ) const { return sensorDigitizationTaps; }
	PropertyI64 getWidthMax( void ) const { return widthMax; }
	PropertyI64 getHeightMax( void ) const { return heightMax; }
	PropertyI64 getWidth( void ) const { return width; }
	PropertyI64 getHeight( void ) const { return height; }
	PropertyI64 getOffsetX( void ) const { return offsetX; }
	PropertyI64 getOffsetY( void ) const { return offsetY; }
	PropertyI64 getmvSensorLineOffsetSelector( void ) const { return mvSensorLineOffsetSelector; }
	PropertyI64 getmvSensorLineOffset( void ) const { return mvSensorLineOffset; }
	PropertyI64 getmvSensorLinePeriod( void ) const { return mvSensorLinePeriod; }
	PropertyI64 getLinePitch( void ) const { return linePitch; }
	PropertyI64 getBinningHorizontal( void ) const { return binningHorizontal; }
	PropertyI64 getBinningVertical( void ) const { return binningVertical; }
	PropertyI64 getDecimationHorizontal( void ) const { return decimationHorizontal; }
	PropertyI64 getDecimationVertical( void ) const { return decimationVertical; }
	PropertyIBoolean getReverseX( void ) const { return reverseX; }
	PropertyIBoolean getReverseY( void ) const { return reverseY; }
	PropertyI64 getPixelFormat( void ) const { return pixelFormat; }
	PropertyI64 getPixelCoding( void ) const { return pixelCoding; }
	PropertyI64 getPixelSize( void ) const { return pixelSize; }
	PropertyI64 getPixelColorFilter( void ) const { return pixelColorFilter; }
	PropertyI64 getPixelDynamicRangeMin( void ) const { return pixelDynamicRangeMin; }
	PropertyI64 getPixelDynamicRangeMax( void ) const { return pixelDynamicRangeMax; }
	PropertyI64 getTestImageSelector( void ) const { return testImageSelector; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category for the acquisition and trigger control features.
///
/// Category for the acquisition and trigger control features.
class AcquisitionControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::AcquisitionControl</b> object.
	explicit AcquisitionControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			acquisitionMode(),
			acquisitionStart(),
			acquisitionStop(),
			acquisitionAbort(),
			acquisitionArm(),
			acquisitionFrameCount(),
			acquisitionBurstFrameCount(),
			acquisitionFrameRate(),
			acquisitionFrameRateAbs(),
			acquisitionFrameRateRaw(),
			acquisitionLineRate(),
			acquisitionLineRateAbs(),
			acquisitionLineRateRaw(),
			acquisitionStatusSelector(),
			acquisitionStatus(),
			triggerSelector(),
			triggerMode(),
			triggerSoftware(),
			triggerSource(),
			triggerActivation(),
			triggerOverlap(),
			triggerDelay(),
			triggerDelayAbs(),
			triggerDelayRaw(),
			triggerDivider(),
			triggerMultiplier(),
			exposureMode(),
			exposureTime(),
			exposureTimeAbs(),
			exposureTimeRaw(),
			exposureAuto(),
			mvShutterMode(),
			mvCompressionKneepoint(),
			mvDefectivePixelEnable(),
			mvExposureAutoLowerLimit(),
			mvExposureAutoUpperLimit(),
			mvExposureAutoSpeed(),
			mvExposureAutoDelayImages(),
			mvExposureAutoAverageGrey(),
			mvExposureAutoHighlightAOI(),
			mvExposureAutoAOIMode(),
			mvExposureAutoOffsetX(),
			mvExposureAutoOffsetY(),
			mvExposureAutoWidth(),
			mvExposureAutoHeight(),
			mvExposureAutoMode(),
			mvSmearReduction(),
			mvAcquisitionMemoryMode(),
			mvPretriggerFrameCount(),
			mvAcquisitionMemoryMaxFrameCount(),
			mvAcquisitionMemoryAOIParameterChanged()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( acquisitionMode, "AcquisitionMode" );
		locator.bindComponent( acquisitionStart, "AcquisitionStart@i" );
		locator.bindComponent( acquisitionStop, "AcquisitionStop@i" );
		locator.bindComponent( acquisitionAbort, "AcquisitionAbort@i" );
		locator.bindComponent( acquisitionArm, "AcquisitionArm@i" );
		locator.bindComponent( acquisitionFrameCount, "AcquisitionFrameCount" );
		locator.bindComponent( acquisitionBurstFrameCount, "AcquisitionBurstFrameCount" );
		locator.bindComponent( acquisitionFrameRate, "AcquisitionFrameRate" );
		locator.bindComponent( acquisitionFrameRateAbs, "AcquisitionFrameRateAbs" );
		locator.bindComponent( acquisitionFrameRateRaw, "AcquisitionFrameRateRaw" );
		locator.bindComponent( acquisitionLineRate, "AcquisitionLineRate" );
		locator.bindComponent( acquisitionLineRateAbs, "AcquisitionLineRateAbs" );
		locator.bindComponent( acquisitionLineRateRaw, "AcquisitionLineRateRaw" );
		locator.bindComponent( acquisitionStatusSelector, "AcquisitionStatusSelector" );
		locator.bindComponent( acquisitionStatus, "AcquisitionStatus" );
		locator.bindComponent( triggerSelector, "TriggerSelector" );
		locator.bindComponent( triggerMode, "TriggerMode" );
		locator.bindComponent( triggerSoftware, "TriggerSoftware@i" );
		locator.bindComponent( triggerSource, "TriggerSource" );
		locator.bindComponent( triggerActivation, "TriggerActivation" );
		locator.bindComponent( triggerOverlap, "TriggerOverlap" );
		locator.bindComponent( triggerDelay, "TriggerDelay" );
		locator.bindComponent( triggerDelayAbs, "TriggerDelayAbs" );
		locator.bindComponent( triggerDelayRaw, "TriggerDelayRaw" );
		locator.bindComponent( triggerDivider, "TriggerDivider" );
		locator.bindComponent( triggerMultiplier, "TriggerMultiplier" );
		locator.bindComponent( exposureMode, "ExposureMode" );
		locator.bindComponent( exposureTime, "ExposureTime" );
		locator.bindComponent( exposureTimeAbs, "ExposureTimeAbs" );
		locator.bindComponent( exposureTimeRaw, "ExposureTimeRaw" );
		locator.bindComponent( exposureAuto, "ExposureAuto" );
		locator.bindComponent( mvShutterMode, "mvShutterMode" );
		locator.bindComponent( mvCompressionKneepoint, "mvCompressionKneepoint" );
		locator.bindComponent( mvDefectivePixelEnable, "mvDefectivePixelEnable" );
		locator.bindComponent( mvExposureAutoLowerLimit, "mvExposureAutoLowerLimit" );
		locator.bindComponent( mvExposureAutoUpperLimit, "mvExposureAutoUpperLimit" );
		locator.bindComponent( mvExposureAutoSpeed, "mvExposureAutoSpeed" );
		locator.bindComponent( mvExposureAutoDelayImages, "mvExposureAutoDelayImages" );
		locator.bindComponent( mvExposureAutoAverageGrey, "mvExposureAutoAverageGrey" );
		locator.bindComponent( mvExposureAutoHighlightAOI, "mvExposureAutoHighlightAOI" );
		locator.bindComponent( mvExposureAutoAOIMode, "mvExposureAutoAOIMode" );
		locator.bindComponent( mvExposureAutoOffsetX, "mvExposureAutoOffsetX" );
		locator.bindComponent( mvExposureAutoOffsetY, "mvExposureAutoOffsetY" );
		locator.bindComponent( mvExposureAutoWidth, "mvExposureAutoWidth" );
		locator.bindComponent( mvExposureAutoHeight, "mvExposureAutoHeight" );
		locator.bindComponent( mvExposureAutoMode, "mvExposureAutoMode" );
		locator.bindComponent( mvSmearReduction, "mvSmearReduction" );
		locator.bindComponent( mvAcquisitionMemoryMode, "mvAcquisitionMemoryMode" );
		locator.bindComponent( mvPretriggerFrameCount, "mvPretriggerFrameCount" );
		locator.bindComponent( mvAcquisitionMemoryMaxFrameCount, "mvAcquisitionMemoryMaxFrameCount" );
		locator.bindComponent( mvAcquisitionMemoryAOIParameterChanged, "mvAcquisitionMemoryAOIParameterChanged" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Sets the acquisition mode of the device.
	///
	/// Sets the acquisition mode of the device. It defines mainly the number of frames to capture during an acquisition and the way the acquisition stops.
	PropertyI64 acquisitionMode;
	/// \brief Starts the Acquisition of the device.
	///
	/// Starts the Acquisition of the device. The number of frames captured is specified by AcquisitionMode.
	Method acquisitionStart;
	/// \brief Stops the Acquisition of the device at the end of the current Frame.
	///
	/// Stops the Acquisition of the device at the end of the current Frame. It is mainly used when AcquisitionMode is Continuous but can be used in any acquisition mode.
	Method acquisitionStop;
	/// \brief Aborts the Acquisition immediately.
	///
	/// Aborts the Acquisition immediately. This will end the capture without completing the current Frame or waiting on a trigger. If no Acquisition is in progress, the command is ignored.
	Method acquisitionAbort;
	/// \brief Arms the device before an AcquisitionStart command.
	///
	/// Arms the device before an AcquisitionStart command. This optional command validates all the current features for consistency and prepares the device for a fast start of the Acquisition.
	Method acquisitionArm;
	/// \brief Number of frames to acquire in MultiFrame Acquisition mode.
	///
	/// Number of frames to acquire in MultiFrame Acquisition mode.
	PropertyI64 acquisitionFrameCount;
	/// \brief Number of frames to acquire for each FrameBurstStart trigger.
	///
	/// Number of frames to acquire for each FrameBurstStart trigger.
	PropertyI64 acquisitionBurstFrameCount;
	/// \brief Controls the acquisition rate (in Hertz) at which the frames are captured.
	///
	/// Controls the acquisition rate (in Hertz) at which the frames are captured.
	PropertyF acquisitionFrameRate;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the rate (in Hertz) at which the Frames are captured when TriggerMode is Off for the Frame trigger.
	PropertyF acquisitionFrameRateAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the rate (in device specific unit) at which the Frames are captured when TriggerMode is Off for the Frame trigger.
	PropertyI64 acquisitionFrameRateRaw;
	/// \brief Controls the rate (in Hertz) at which the Lines in a Frame are captured.
	///
	/// Controls the rate (in Hertz) at which the Lines in a Frame are captured.
	PropertyF acquisitionLineRate;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the rate (in Hertz) at which the Lines in a Frame are captured when TriggerMode is Off for the Line trigger.
	PropertyF acquisitionLineRateAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the rate (in device specific unit) at which the Lines in a Frame are captured when TriggerMode is Off for the Line trigger.
	PropertyI64 acquisitionLineRateRaw;
	/// \brief Selects the internal acquisition signal to read using AcquisitionStatus.
	///
	/// Selects the internal acquisition signal to read using AcquisitionStatus.
	PropertyI64 acquisitionStatusSelector;
	/// \brief Reads the state of the internal acquisition signal selected using AcquisitionStatusSelector.
	///
	/// Reads the state of the internal acquisition signal selected using AcquisitionStatusSelector.
	PropertyIBoolean acquisitionStatus;
	/// \brief Selects the type of trigger to configure.
	///
	/// Selects the type of trigger to configure.
	PropertyI64 triggerSelector;
	/// \brief Controls if the selected trigger is active.
	///
	/// Controls if the selected trigger is active.
	PropertyI64 triggerMode;
	/// \brief Generates an internal trigger.
	///
	/// Generates an internal trigger. TriggerSource must be set to Software.
	Method triggerSoftware;
	/// \brief Specifies the internal signal or physical input Line to use as the trigger source.
	///
	/// Specifies the internal signal or physical input Line to use as the trigger source. The selected trigger must have its TriggerMode set to On.
	PropertyI64 triggerSource;
	/// \brief Specifies the activation mode of the trigger.
	///
	/// Specifies the activation mode of the trigger.
	PropertyI64 triggerActivation;
	/// \brief Specifies the type trigger overlap permitted with the previous frame.
	///
	/// Specifies the type trigger overlap permitted with the previous frame. This defines when a valid trigger will be accepted (or latched) for a new frame.
	PropertyI64 triggerOverlap;
	/// \brief Specifies the delay in microseconds (us) to apply after the trigger reception before activating it.
	///
	/// Specifies the delay in microseconds (us) to apply after the trigger reception before activating it.
	PropertyF triggerDelay;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It specifies the absolute delay in microseconds (us) to apply after the trigger reception before effectively activating it. TriggerDelayRaw must reflect the state of TriggerDelayAbs when they are both supported.
	PropertyF triggerDelayAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. TriggerDelayRaw specifies the delay in device-specific unit to apply after the trigger reception before effectively activating it. TriggerDelayAbs must reflect the state of TriggerDelayRaw when they are both supported.
	PropertyI64 triggerDelayRaw;
	/// \brief Specifies a division factor for the incoming trigger pulses.
	///
	/// Specifies a division factor for the incoming trigger pulses.
	PropertyI64 triggerDivider;
	/// \brief Specifies a multiplication factor for the incoming trigger pulses.
	///
	/// Specifies a multiplication factor for the incoming trigger pulses. It is used generally used in conjunction with TriggerDivider to control the ratio of triggers that are accepted.
	PropertyI64 triggerMultiplier;
	/// \brief Sets the operation mode of the Exposure (or shutter).
	///
	/// Sets the operation mode of the Exposure (or shutter).
	PropertyI64 exposureMode;
	/// \brief Sets the Exposure time (in microseconds) when ExposureMode is Timed and ExposureAuto is Off.
	///
	/// Sets the Exposure time (in microseconds) when ExposureMode is Timed and ExposureAuto is Off. This controls the duration where the photosensitive cells are exposed to light.
	PropertyF exposureTime;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It is used to set the Exposure time (in microseconds) when ExposureMode is Timed. This controls the duration where the photosensitive cells are exposed to light.
	PropertyF exposureTimeAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It can used to set the Exposure time in device-specific unit when ExposureMode is Timed. This controls the duration where the photosensitive cells are exposed to light.
	PropertyI64 exposureTimeRaw;
	/// \brief Sets the automatic exposure mode when ExposureMode is Timed.
	///
	/// Sets the automatic exposure mode when ExposureMode is Timed. The exact algorithm used to implement this control is device-specific.
	PropertyI64 exposureAuto;
	/// \brief Selects the shutter mode of the sensor.
	///
	/// Selects the shutter mode of the sensor.
	PropertyI64 mvShutterMode;
	/// \brief Kneepoint of 10 to 8 bit compression.
	///
	/// Kneepoint of 10 to 8 bit compression.
	PropertyI64 mvCompressionKneepoint;
	/// \brief Activates the sensor's defective pixel correction.
	///
	/// Activates the sensor's defective pixel correction.
	PropertyIBoolean mvDefectivePixelEnable;
	/// \brief The lower limit of the exposure time in auto exposure mode.
	///
	/// The lower limit of the exposure time in auto exposure mode.
	PropertyF mvExposureAutoLowerLimit;
	/// \brief The upper limit of the exposure time in auto exposure mode.
	///
	/// The upper limit of the exposure time in auto exposure mode.
	PropertyF mvExposureAutoUpperLimit;
	/// \brief Determines the increment or decrement size of exposure value from frame to frame.
	///
	/// Determines the increment or decrement size of exposure value from frame to frame.
	PropertyI64 mvExposureAutoSpeed;
	/// \brief The number of frames that the AEC must skip before updating the exposure register.
	///
	/// The number of frames that the AEC must skip before updating the exposure register.
	PropertyI64 mvExposureAutoDelayImages;
	/// \brief Common desired average grey value (in percent) used for Auto Gain Control(AGC) and Auto Exposure Control(AEC).
	///
	/// Common desired average grey value (in percent) used for auto gain control(AGC) and auto exposure control (AEC).
	PropertyI64 mvExposureAutoAverageGrey;
	/// \brief Highlight auto control AOI to check AOI settings. Switch off for normal operation.
	///
	/// Highlight auto control AOI to check AOI settings. Switch off for normal operation.
	PropertyI64 mvExposureAutoHighlightAOI;
	/// \brief Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvExposureAutoAOIMode;
	/// \brief Common AOI XOffset used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI XOffset used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvExposureAutoOffsetX;
	/// \brief Common AOI YOffset used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI YOffset used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvExposureAutoOffsetY;
	/// \brief Common AOI Width used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Width used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvExposureAutoWidth;
	/// \brief Common AOI Height used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Height used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvExposureAutoHeight;
	/// \brief Selects the common auto mode for gain and exposure.
	///
	/// Selects the common auto mode for gain and exposure.
	PropertyI64 mvExposureAutoMode;
	/// \brief Smear reduction in triggered and nonoverlapped mode.
	///
	/// Smear reduction in triggered and nonoverlapped mode.
	PropertyI64 mvSmearReduction;
	/// \brief mvRecord is used to store frames in memory. mvPlayback transfers stored frames. mvPretrigger stores frames in memory to be transfered after trigger.
	///
	/// mvRecord is used to store frames in memory. mvPlayback transfers stored frames. mvPretrigger stores frames in memory to be transfered after trigger.
	PropertyI64 mvAcquisitionMemoryMode;
	/// \brief Number of frames to acquire before the occurence of an AcquisitionStart or AcquisitionActive trigger.
	///
	/// Number of frames to acquire before the occurence of an AcquisitionStart or AcquisitionActive trigger.
	PropertyI64 mvPretriggerFrameCount;
	/// \brief Max number of frames to record.
	///
	/// Max number of frames to record.
	PropertyI64 mvAcquisitionMemoryMaxFrameCount;
	/// \brief AOI and/or binning parameter changed after last Acquisition.
	///
	/// AOI and/or binning parameter changed after last Acquisition.
	PropertyI64 mvAcquisitionMemoryAOIParameterChanged;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getAcquisitionMode( void ) const { return acquisitionMode; }
	Method getAcquisitionStart( void ) const { return acquisitionStart; }
	Method getAcquisitionStop( void ) const { return acquisitionStop; }
	Method getAcquisitionAbort( void ) const { return acquisitionAbort; }
	Method getAcquisitionArm( void ) const { return acquisitionArm; }
	PropertyI64 getAcquisitionFrameCount( void ) const { return acquisitionFrameCount; }
	PropertyI64 getAcquisitionBurstFrameCount( void ) const { return acquisitionBurstFrameCount; }
	PropertyF getAcquisitionFrameRate( void ) const { return acquisitionFrameRate; }
	PropertyF getAcquisitionFrameRateAbs( void ) const { return acquisitionFrameRateAbs; }
	PropertyI64 getAcquisitionFrameRateRaw( void ) const { return acquisitionFrameRateRaw; }
	PropertyF getAcquisitionLineRate( void ) const { return acquisitionLineRate; }
	PropertyF getAcquisitionLineRateAbs( void ) const { return acquisitionLineRateAbs; }
	PropertyI64 getAcquisitionLineRateRaw( void ) const { return acquisitionLineRateRaw; }
	PropertyI64 getAcquisitionStatusSelector( void ) const { return acquisitionStatusSelector; }
	PropertyIBoolean getAcquisitionStatus( void ) const { return acquisitionStatus; }
	PropertyI64 getTriggerSelector( void ) const { return triggerSelector; }
	PropertyI64 getTriggerMode( void ) const { return triggerMode; }
	Method getTriggerSoftware( void ) const { return triggerSoftware; }
	PropertyI64 getTriggerSource( void ) const { return triggerSource; }
	PropertyI64 getTriggerActivation( void ) const { return triggerActivation; }
	PropertyI64 getTriggerOverlap( void ) const { return triggerOverlap; }
	PropertyF getTriggerDelay( void ) const { return triggerDelay; }
	PropertyF getTriggerDelayAbs( void ) const { return triggerDelayAbs; }
	PropertyI64 getTriggerDelayRaw( void ) const { return triggerDelayRaw; }
	PropertyI64 getTriggerDivider( void ) const { return triggerDivider; }
	PropertyI64 getTriggerMultiplier( void ) const { return triggerMultiplier; }
	PropertyI64 getExposureMode( void ) const { return exposureMode; }
	PropertyF getExposureTime( void ) const { return exposureTime; }
	PropertyF getExposureTimeAbs( void ) const { return exposureTimeAbs; }
	PropertyI64 getExposureTimeRaw( void ) const { return exposureTimeRaw; }
	PropertyI64 getExposureAuto( void ) const { return exposureAuto; }
	PropertyI64 getmvShutterMode( void ) const { return mvShutterMode; }
	PropertyI64 getmvCompressionKneepoint( void ) const { return mvCompressionKneepoint; }
	PropertyIBoolean getmvDefectivePixelEnable( void ) const { return mvDefectivePixelEnable; }
	PropertyF getmvExposureAutoLowerLimit( void ) const { return mvExposureAutoLowerLimit; }
	PropertyF getmvExposureAutoUpperLimit( void ) const { return mvExposureAutoUpperLimit; }
	PropertyI64 getmvExposureAutoSpeed( void ) const { return mvExposureAutoSpeed; }
	PropertyI64 getmvExposureAutoDelayImages( void ) const { return mvExposureAutoDelayImages; }
	PropertyI64 getmvExposureAutoAverageGrey( void ) const { return mvExposureAutoAverageGrey; }
	PropertyI64 getmvExposureAutoHighlightAOI( void ) const { return mvExposureAutoHighlightAOI; }
	PropertyI64 getmvExposureAutoAOIMode( void ) const { return mvExposureAutoAOIMode; }
	PropertyI64 getmvExposureAutoOffsetX( void ) const { return mvExposureAutoOffsetX; }
	PropertyI64 getmvExposureAutoOffsetY( void ) const { return mvExposureAutoOffsetY; }
	PropertyI64 getmvExposureAutoWidth( void ) const { return mvExposureAutoWidth; }
	PropertyI64 getmvExposureAutoHeight( void ) const { return mvExposureAutoHeight; }
	PropertyI64 getmvExposureAutoMode( void ) const { return mvExposureAutoMode; }
	PropertyI64 getmvSmearReduction( void ) const { return mvSmearReduction; }
	PropertyI64 getmvAcquisitionMemoryMode( void ) const { return mvAcquisitionMemoryMode; }
	PropertyI64 getmvPretriggerFrameCount( void ) const { return mvPretriggerFrameCount; }
	PropertyI64 getmvAcquisitionMemoryMaxFrameCount( void ) const { return mvAcquisitionMemoryMaxFrameCount; }
	PropertyI64 getmvAcquisitionMemoryAOIParameterChanged( void ) const { return mvAcquisitionMemoryAOIParameterChanged; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the digital input and output control features.
///
/// Category that contains the digital input and output control features.
class DigitalIOControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::DigitalIOControl</b> object.
	explicit DigitalIOControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			lineSelector(),
			lineMode(),
			lineInverter(),
			lineStatus(),
			lineStatusAll(),
			lineSource(),
			lineFormat(),
			userOutputSelector(),
			userOutputValue(),
			userOutputValueAll(),
			userOutputValueAllMask(),
			mvLineDebounceTimeRisingEdge(),
			mvLineDebounceTimeFallingEdge()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( lineSelector, "LineSelector" );
		locator.bindComponent( lineMode, "LineMode" );
		locator.bindComponent( lineInverter, "LineInverter" );
		locator.bindComponent( lineStatus, "LineStatus" );
		locator.bindComponent( lineStatusAll, "LineStatusAll" );
		locator.bindComponent( lineSource, "LineSource" );
		locator.bindComponent( lineFormat, "LineFormat" );
		locator.bindComponent( userOutputSelector, "UserOutputSelector" );
		locator.bindComponent( userOutputValue, "UserOutputValue" );
		locator.bindComponent( userOutputValueAll, "UserOutputValueAll" );
		locator.bindComponent( userOutputValueAllMask, "UserOutputValueAllMask" );
		locator.bindComponent( mvLineDebounceTimeRisingEdge, "mvLineDebounceTimeRisingEdge" );
		locator.bindComponent( mvLineDebounceTimeFallingEdge, "mvLineDebounceTimeFallingEdge" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects the physical line (or pin) of the external device connector to configure.
	///
	/// Selects the physical line (or pin) of the external device connector to configure.
	PropertyI64 lineSelector;
	/// \brief Controls if the physical Line is used to Input or Output a signal.
	///
	/// Controls if the physical Line is used to Input or Output a signal.
	PropertyI64 lineMode;
	/// \brief Controls the invertion of the signal of the selected input or output Line.
	///
	/// Controls the invertion of the signal of the selected input or output Line.
	PropertyIBoolean lineInverter;
	/// \brief Returns the current status of the selected input or output Line.
	///
	/// Returns the current status of the selected input or output Line.
	PropertyIBoolean lineStatus;
	/// \brief Returns the current status of all available Line signals at time of polling in a single bitfield.
	///
	/// Returns the current status of all available Line signals at time of polling in a single bitfield.
	PropertyI64 lineStatusAll;
	/// \brief Selects which internal acquisition or I/O source signal to output on the selected Line.
	///
	/// Selects which internal acquisition or I/O source signal to output on the selected Line. LineMode must be Output.
	PropertyI64 lineSource;
	/// \brief Controls the current electrical format of the selected physical input or output Line.
	///
	/// Controls the current electrical format of the selected physical input or output Line.
	PropertyI64 lineFormat;
	/// \brief Selects which bit of the User Output register will be set by UserOutputValue.
	///
	/// Selects which bit of the User Output register will be set by UserOutputValue.
	PropertyI64 userOutputSelector;
	/// \brief Sets the value of the bit selected by UserOutputSelector.
	///
	/// Sets the value of the bit selected by UserOutputSelector.
	PropertyIBoolean userOutputValue;
	/// \brief Sets the value of all the bits of the User Output register.
	///
	/// Sets the value of all the bits of the User Output register. It is subject to the UserOutputValueAllMask.
	PropertyI64 userOutputValueAll;
	/// \brief Sets the write mask to apply to the value specified by UserOutputValueAll before writing it in the User Output register.
	///
	/// Sets the write mask to apply to the value specified by UserOutputValueAll before writing it in the User Output register. If the UserOutputValueAllMask feature is present, setting the user Output register using UserOutputValueAll will only change the bits that have a corresponding bit in the mask set to one.
	PropertyI64 userOutputValueAllMask;
	/// \brief Sets the debounce time in micro seconds for low to high transitions.
	///
	/// Sets the debounce time in micro seconds for low to high transitions.
	PropertyI64 mvLineDebounceTimeRisingEdge;
	/// \brief Sets the debounce time in micro seconds for high to low transitions.
	///
	/// Sets the debounce time in micro seconds for high to low transitions.
	PropertyI64 mvLineDebounceTimeFallingEdge;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getLineSelector( void ) const { return lineSelector; }
	PropertyI64 getLineMode( void ) const { return lineMode; }
	PropertyIBoolean getLineInverter( void ) const { return lineInverter; }
	PropertyIBoolean getLineStatus( void ) const { return lineStatus; }
	PropertyI64 getLineStatusAll( void ) const { return lineStatusAll; }
	PropertyI64 getLineSource( void ) const { return lineSource; }
	PropertyI64 getLineFormat( void ) const { return lineFormat; }
	PropertyI64 getUserOutputSelector( void ) const { return userOutputSelector; }
	PropertyIBoolean getUserOutputValue( void ) const { return userOutputValue; }
	PropertyI64 getUserOutputValueAll( void ) const { return userOutputValueAll; }
	PropertyI64 getUserOutputValueAllMask( void ) const { return userOutputValueAllMask; }
	PropertyI64 getmvLineDebounceTimeRisingEdge( void ) const { return mvLineDebounceTimeRisingEdge; }
	PropertyI64 getmvLineDebounceTimeFallingEdge( void ) const { return mvLineDebounceTimeFallingEdge; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the Counter and Timer control features.
///
/// Category that contains the Counter and Timer control features.
class CounterAndTimerControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::CounterAndTimerControl</b> object.
	explicit CounterAndTimerControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			counterSelector(),
			counterEventSource(),
			counterEventActivation(),
			counterResetSource(),
			counterResetActivation(),
			counterReset(),
			counterValue(),
			counterValueAtReset(),
			counterDuration(),
			counterStatus(),
			counterTriggerSource(),
			counterTriggerActivation(),
			timerSelector(),
			timerDuration(),
			timerDurationAbs(),
			timerDurationRaw(),
			timerDelay(),
			timerDelayAbs(),
			timerDelayRaw(),
			timerReset(),
			timerValue(),
			timerValueAbs(),
			timerValueRaw(),
			timerStatus(),
			timerTriggerSource(),
			timerTriggerActivation()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( counterSelector, "CounterSelector" );
		locator.bindComponent( counterEventSource, "CounterEventSource" );
		locator.bindComponent( counterEventActivation, "CounterEventActivation" );
		locator.bindComponent( counterResetSource, "CounterResetSource" );
		locator.bindComponent( counterResetActivation, "CounterResetActivation" );
		locator.bindComponent( counterReset, "CounterReset@i" );
		locator.bindComponent( counterValue, "CounterValue" );
		locator.bindComponent( counterValueAtReset, "CounterValueAtReset" );
		locator.bindComponent( counterDuration, "CounterDuration" );
		locator.bindComponent( counterStatus, "CounterStatus" );
		locator.bindComponent( counterTriggerSource, "CounterTriggerSource" );
		locator.bindComponent( counterTriggerActivation, "CounterTriggerActivation" );
		locator.bindComponent( timerSelector, "TimerSelector" );
		locator.bindComponent( timerDuration, "TimerDuration" );
		locator.bindComponent( timerDurationAbs, "TimerDurationAbs" );
		locator.bindComponent( timerDurationRaw, "TimerDurationRaw" );
		locator.bindComponent( timerDelay, "TimerDelay" );
		locator.bindComponent( timerDelayAbs, "TimerDelayAbs" );
		locator.bindComponent( timerDelayRaw, "TimerDelayRaw" );
		locator.bindComponent( timerReset, "TimerReset@i" );
		locator.bindComponent( timerValue, "TimerValue" );
		locator.bindComponent( timerValueAbs, "TimerValueAbs" );
		locator.bindComponent( timerValueRaw, "TimerValueRaw" );
		locator.bindComponent( timerStatus, "TimerStatus" );
		locator.bindComponent( timerTriggerSource, "TimerTriggerSource" );
		locator.bindComponent( timerTriggerActivation, "TimerTriggerActivation" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects which Counter to configure.
	///
	/// Selects which Counter to configure.
	PropertyI64 counterSelector;
	/// \brief Select the events that will be the source to increment the Counter.
	///
	/// Select the events that will be the source to increment the Counter.
	PropertyI64 counterEventSource;
	/// \brief Selects the Activation mode Event Source signal.
	///
	/// Selects the Activation mode Event Source signal.
	PropertyI64 counterEventActivation;
	/// \brief Selects the signals that will be the source to reset the Counter.
	///
	/// Selects the signals that will be the source to reset the Counter.
	PropertyI64 counterResetSource;
	/// \brief Selects the Activation mode of the Counter Reset Source signal.
	///
	/// Selects the Activation mode of the Counter Reset Source signal.
	PropertyI64 counterResetActivation;
	/// \brief Does a software reset of the selected Counter and starts it.
	///
	/// Does a software reset of the selected Counter and starts it. The counter starts counting events immediately after the reset unless a Counter trigger is active. CounterReset can be used to reset the Counter independently from the CounterResetSource. To disable the counter temporarily, set CounterEventSource to Off.
	Method counterReset;
	/// \brief Reads or writes the current value of the selected Counter.
	///
	/// Reads or writes the current value of the selected Counter.
	PropertyI64 counterValue;
	/// \brief Reads the value of the selected Counter when it was reset by a trigger or by an explicit CounterReset command.
	///
	/// Reads the value of the selected Counter when it was reset by a trigger or by an explicit CounterReset command.
	PropertyI64 counterValueAtReset;
	/// \brief Sets the duration (or number of events) before the CounterEnd event is generated.
	///
	/// Sets the duration (or number of events) before the CounterEnd event is generated.
	PropertyI64 counterDuration;
	/// \brief Returns the current state of the Counter.
	///
	/// Returns the current state of the Counter.
	PropertyI64 counterStatus;
	/// \brief Selects the source to start the Counter.
	///
	/// Selects the source to start the Counter. CounterTriggerSource can take any of the following values:
	PropertyI64 counterTriggerSource;
	/// \brief Selects the activation mode of the trigger to start the Counter.
	///
	/// Selects the activation mode of the trigger to start the Counter.
	PropertyI64 counterTriggerActivation;
	/// \brief Selects which Timer to configure.
	///
	/// Selects which Timer to configure.
	PropertyI64 timerSelector;
	/// \brief Sets the duration (in microseconds) of the Timer pulse.
	///
	/// Sets the duration (in microseconds) of the Timer pulse.
	PropertyF timerDuration;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It sets the duration (in microseconds) of the Timer pulse.
	PropertyF timerDurationAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It sets the duration in device-specific unit of the Timer pulse.
	PropertyI64 timerDurationRaw;
	/// \brief Sets the duration (in microseconds) of the delay to apply at the reception of a trigger before starting the Timer.
	///
	/// Sets the duration (in microseconds) of the delay to apply at the reception of a trigger before starting the Timer.
	PropertyF timerDelay;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. This feature sets the duration (in microseconds) of the delay to apply after the reception of a trigger before starting the Timer.
	PropertyF timerDelayAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It sets the duration in device-specific unit of the delay to apply after the reception of a trigger before starting the Timer.
	PropertyI64 timerDelayRaw;
	/// \brief Does a software reset of the selected timer and starts it.
	///
	/// Does a software reset of the selected timer and starts it. The timer starts counting events immediately after the reset unless a timer trigger is active.
	Method timerReset;
	/// \brief Reads or writes the current value (in microseconds) of the selected Timer.
	///
	/// Reads or writes the current value (in microseconds) of the selected Timer.
	PropertyF timerValue;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It returns the current value (in microseconds) of the selected Timer.
	PropertyF timerValueAbs;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. This feature is used to read the current value in device-specific unit of the selected Timer.
	PropertyI64 timerValueRaw;
	/// \brief Returns the current state of the Timer.
	///
	/// Returns the current state of the Timer.
	PropertyI64 timerStatus;
	/// \brief Selects the source of the trigger to start the Timer.
	///
	/// Selects the source of the trigger to start the Timer.
	PropertyI64 timerTriggerSource;
	/// \brief Selects the activation mode of the trigger to start the Timer.
	///
	/// Selects the activation mode of the trigger to start the Timer.
	PropertyI64 timerTriggerActivation;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getCounterSelector( void ) const { return counterSelector; }
	PropertyI64 getCounterEventSource( void ) const { return counterEventSource; }
	PropertyI64 getCounterEventActivation( void ) const { return counterEventActivation; }
	PropertyI64 getCounterResetSource( void ) const { return counterResetSource; }
	PropertyI64 getCounterResetActivation( void ) const { return counterResetActivation; }
	Method getCounterReset( void ) const { return counterReset; }
	PropertyI64 getCounterValue( void ) const { return counterValue; }
	PropertyI64 getCounterValueAtReset( void ) const { return counterValueAtReset; }
	PropertyI64 getCounterDuration( void ) const { return counterDuration; }
	PropertyI64 getCounterStatus( void ) const { return counterStatus; }
	PropertyI64 getCounterTriggerSource( void ) const { return counterTriggerSource; }
	PropertyI64 getCounterTriggerActivation( void ) const { return counterTriggerActivation; }
	PropertyI64 getTimerSelector( void ) const { return timerSelector; }
	PropertyF getTimerDuration( void ) const { return timerDuration; }
	PropertyF getTimerDurationAbs( void ) const { return timerDurationAbs; }
	PropertyI64 getTimerDurationRaw( void ) const { return timerDurationRaw; }
	PropertyF getTimerDelay( void ) const { return timerDelay; }
	PropertyF getTimerDelayAbs( void ) const { return timerDelayAbs; }
	PropertyI64 getTimerDelayRaw( void ) const { return timerDelayRaw; }
	Method getTimerReset( void ) const { return timerReset; }
	PropertyF getTimerValue( void ) const { return timerValue; }
	PropertyF getTimerValueAbs( void ) const { return timerValueAbs; }
	PropertyI64 getTimerValueRaw( void ) const { return timerValueRaw; }
	PropertyI64 getTimerStatus( void ) const { return timerStatus; }
	PropertyI64 getTimerTriggerSource( void ) const { return timerTriggerSource; }
	PropertyI64 getTimerTriggerActivation( void ) const { return timerTriggerActivation; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains Event control features.
///
/// Category that contains Event control features.
class EventControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::EventControl</b> object.
	explicit EventControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			eventSelector(),
			eventNotification(),
			eventAcquisitionTrigger(),
			eventAcquisitionTriggerTimestamp(),
			eventAcquisitionTriggerFrameID(),
			eventAcquisitionStart(),
			eventAcquisitionStartTimestamp(),
			eventAcquisitionStartFrameID(),
			eventAcquisitionEnd(),
			eventAcquisitionEndTimestamp(),
			eventAcquisitionEndFrameID(),
			eventAcquisitionTransferStart(),
			eventAcquisitionTransferStartTimestamp(),
			eventAcquisitionTransferStartFrameID(),
			eventAcquisitionTransferEnd(),
			eventAcquisitionTransferEndTimestamp(),
			eventAcquisitionTransferEndFrameID(),
			eventAcquisitionError(),
			eventAcquisitionErrorTimestamp(),
			eventAcquisitionErrorFrameID(),
			eventFrameTrigger(),
			eventFrameTriggerTimestamp(),
			eventFrameTriggerFrameID(),
			eventFrameStart(),
			eventFrameStartTimestamp(),
			eventFrameStartFrameID(),
			eventFrameEnd(),
			eventFrameEndTimestamp(),
			eventFrameEndFrameID(),
			eventFrameBurstStart(),
			eventFrameBurstStartTimestamp(),
			eventFrameBurstStartFrameID(),
			eventFrameBurstEnd(),
			eventFrameBurstEndTimestamp(),
			eventFrameBurstEndFrameID(),
			eventFrameTransferStart(),
			eventFrameTransferStartTimestamp(),
			eventFrameTransferStartFrameID(),
			eventFrameTransferEnd(),
			eventFrameTransferEndTimestamp(),
			eventFrameTransferEndFrameID(),
			eventExposureStart(),
			eventExposureStartTimestamp(),
			eventExposureStartFrameID(),
			eventExposureEnd(),
			eventExposureEndTimestamp(),
			eventExposureEndFrameID(),
			eventCounter1Start(),
			eventCounter1StartTimestamp(),
			eventCounter1StartFrameID(),
			eventCounter2Start(),
			eventCounter2StartTimestamp(),
			eventCounter2StartFrameID(),
			eventCounter1End(),
			eventCounter1EndTimestamp(),
			eventCounter1EndFrameID(),
			eventCounter2End(),
			eventCounter2EndTimestamp(),
			eventCounter2EndFrameID(),
			eventTimer1Start(),
			eventTimer1StartTimestamp(),
			eventTimer1StartFrameID(),
			eventTimer2Start(),
			eventTimer2StartTimestamp(),
			eventTimer2StartFrameID(),
			eventTimer1End(),
			eventTimer1EndTimestamp(),
			eventTimer1EndFrameID(),
			eventTimer2End(),
			eventTimer2EndTimestamp(),
			eventTimer2EndFrameID(),
			eventLine0RisingEdge(),
			eventLine0RisingEdgeTimestamp(),
			eventLine0RisingEdgeFrameID(),
			eventLine1RisingEdge(),
			eventLine1RisingEdgeTimestamp(),
			eventLine1RisingEdgeFrameID(),
			eventLine2RisingEdge(),
			eventLine2RisingEdgeTimestamp(),
			eventLine2RisingEdgeFrameID(),
			eventLine3RisingEdge(),
			eventLine3RisingEdgeTimestamp(),
			eventLine3RisingEdgeFrameID(),
			eventLine0FallingEdge(),
			eventLine0FallingEdgeTimestamp(),
			eventLine0FallingEdgeFrameID(),
			eventLine1FallingEdge(),
			eventLine1FallingEdgeTimestamp(),
			eventLine1FallingEdgeFrameID(),
			eventLine2FallingEdge(),
			eventLine2FallingEdgeTimestamp(),
			eventLine2FallingEdgeFrameID(),
			eventLine3FallingEdge(),
			eventLine3FallingEdgeTimestamp(),
			eventLine3FallingEdgeFrameID(),
			eventLine0AnyEdge(),
			eventLine0AnyEdgeTimestamp(),
			eventLine0AnyEdgeFrameID(),
			eventLine1AnyEdge(),
			eventLine1AnyEdgeTimestamp(),
			eventLine1AnyEdgeFrameID(),
			eventLine2AnyEdge(),
			eventLine2AnyEdgeTimestamp(),
			eventLine2AnyEdgeFrameID(),
			eventLine3AnyEdge(),
			eventLine3AnyEdgeTimestamp(),
			eventLine3AnyEdgeFrameID(),
			eventError(),
			eventErrorTimestamp(),
			eventErrorFrameID(),
			eventErrorCode()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( eventSelector, "EventSelector" );
		locator.bindComponent( eventNotification, "EventNotification" );
		locator.bindComponent( eventAcquisitionTrigger, "EventAcquisitionTrigger" );
		locator.bindComponent( eventAcquisitionTriggerTimestamp, "EventAcquisitionTriggerTimestamp" );
		locator.bindComponent( eventAcquisitionTriggerFrameID, "EventAcquisitionTriggerFrameID" );
		locator.bindComponent( eventAcquisitionStart, "EventAcquisitionStart" );
		locator.bindComponent( eventAcquisitionStartTimestamp, "EventAcquisitionStartTimestamp" );
		locator.bindComponent( eventAcquisitionStartFrameID, "EventAcquisitionStartFrameID" );
		locator.bindComponent( eventAcquisitionEnd, "EventAcquisitionEnd" );
		locator.bindComponent( eventAcquisitionEndTimestamp, "EventAcquisitionEndTimestamp" );
		locator.bindComponent( eventAcquisitionEndFrameID, "EventAcquisitionEndFrameID" );
		locator.bindComponent( eventAcquisitionTransferStart, "EventAcquisitionTransferStart" );
		locator.bindComponent( eventAcquisitionTransferStartTimestamp, "EventAcquisitionTransferStartTimestamp" );
		locator.bindComponent( eventAcquisitionTransferStartFrameID, "EventAcquisitionTransferStartFrameID" );
		locator.bindComponent( eventAcquisitionTransferEnd, "EventAcquisitionTransferEnd" );
		locator.bindComponent( eventAcquisitionTransferEndTimestamp, "EventAcquisitionTransferEndTimestamp" );
		locator.bindComponent( eventAcquisitionTransferEndFrameID, "EventAcquisitionTransferEndFrameID" );
		locator.bindComponent( eventAcquisitionError, "EventAcquisitionError" );
		locator.bindComponent( eventAcquisitionErrorTimestamp, "EventAcquisitionErrorTimestamp" );
		locator.bindComponent( eventAcquisitionErrorFrameID, "EventAcquisitionErrorFrameID" );
		locator.bindComponent( eventFrameTrigger, "EventFrameTrigger" );
		locator.bindComponent( eventFrameTriggerTimestamp, "EventFrameTriggerTimestamp" );
		locator.bindComponent( eventFrameTriggerFrameID, "EventFrameTriggerFrameID" );
		locator.bindComponent( eventFrameStart, "EventFrameStart" );
		locator.bindComponent( eventFrameStartTimestamp, "EventFrameStartTimestamp" );
		locator.bindComponent( eventFrameStartFrameID, "EventFrameStartFrameID" );
		locator.bindComponent( eventFrameEnd, "EventFrameEnd" );
		locator.bindComponent( eventFrameEndTimestamp, "EventFrameEndTimestamp" );
		locator.bindComponent( eventFrameEndFrameID, "EventFrameEndFrameID" );
		locator.bindComponent( eventFrameBurstStart, "EventFrameBurstStart" );
		locator.bindComponent( eventFrameBurstStartTimestamp, "EventFrameBurstStartTimestamp" );
		locator.bindComponent( eventFrameBurstStartFrameID, "EventFrameBurstStartFrameID" );
		locator.bindComponent( eventFrameBurstEnd, "EventFrameBurstEnd" );
		locator.bindComponent( eventFrameBurstEndTimestamp, "EventFrameBurstEndTimestamp" );
		locator.bindComponent( eventFrameBurstEndFrameID, "EventFrameBurstEndFrameID" );
		locator.bindComponent( eventFrameTransferStart, "EventFrameTransferStart" );
		locator.bindComponent( eventFrameTransferStartTimestamp, "EventFrameTransferStartTimestamp" );
		locator.bindComponent( eventFrameTransferStartFrameID, "EventFrameTransferStartFrameID" );
		locator.bindComponent( eventFrameTransferEnd, "EventFrameTransferEnd" );
		locator.bindComponent( eventFrameTransferEndTimestamp, "EventFrameTransferEndTimestamp" );
		locator.bindComponent( eventFrameTransferEndFrameID, "EventFrameTransferEndFrameID" );
		locator.bindComponent( eventExposureStart, "EventExposureStart" );
		locator.bindComponent( eventExposureStartTimestamp, "EventExposureStartTimestamp" );
		locator.bindComponent( eventExposureStartFrameID, "EventExposureStartFrameID" );
		locator.bindComponent( eventExposureEnd, "EventExposureEnd" );
		locator.bindComponent( eventExposureEndTimestamp, "EventExposureEndTimestamp" );
		locator.bindComponent( eventExposureEndFrameID, "EventExposureEndFrameID" );
		locator.bindComponent( eventCounter1Start, "EventCounter1Start" );
		locator.bindComponent( eventCounter1StartTimestamp, "EventCounter1StartTimestamp" );
		locator.bindComponent( eventCounter1StartFrameID, "EventCounter1StartFrameID" );
		locator.bindComponent( eventCounter2Start, "EventCounter2Start" );
		locator.bindComponent( eventCounter2StartTimestamp, "EventCounter2StartTimestamp" );
		locator.bindComponent( eventCounter2StartFrameID, "EventCounter2StartFrameID" );
		locator.bindComponent( eventCounter1End, "EventCounter1End" );
		locator.bindComponent( eventCounter1EndTimestamp, "EventCounter1EndTimestamp" );
		locator.bindComponent( eventCounter1EndFrameID, "EventCounter1EndFrameID" );
		locator.bindComponent( eventCounter2End, "EventCounter2End" );
		locator.bindComponent( eventCounter2EndTimestamp, "EventCounter2EndTimestamp" );
		locator.bindComponent( eventCounter2EndFrameID, "EventCounter2EndFrameID" );
		locator.bindComponent( eventTimer1Start, "EventTimer1Start" );
		locator.bindComponent( eventTimer1StartTimestamp, "EventTimer1StartTimestamp" );
		locator.bindComponent( eventTimer1StartFrameID, "EventTimer1StartFrameID" );
		locator.bindComponent( eventTimer2Start, "EventTimer2Start" );
		locator.bindComponent( eventTimer2StartTimestamp, "EventTimer2StartTimestamp" );
		locator.bindComponent( eventTimer2StartFrameID, "EventTimer2StartFrameID" );
		locator.bindComponent( eventTimer1End, "EventTimer1End" );
		locator.bindComponent( eventTimer1EndTimestamp, "EventTimer1EndTimestamp" );
		locator.bindComponent( eventTimer1EndFrameID, "EventTimer1EndFrameID" );
		locator.bindComponent( eventTimer2End, "EventTimer2End" );
		locator.bindComponent( eventTimer2EndTimestamp, "EventTimer2EndTimestamp" );
		locator.bindComponent( eventTimer2EndFrameID, "EventTimer2EndFrameID" );
		locator.bindComponent( eventLine0RisingEdge, "EventLine0RisingEdge" );
		locator.bindComponent( eventLine0RisingEdgeTimestamp, "EventLine0RisingEdgeTimestamp" );
		locator.bindComponent( eventLine0RisingEdgeFrameID, "EventLine0RisingEdgeFrameID" );
		locator.bindComponent( eventLine1RisingEdge, "EventLine1RisingEdge" );
		locator.bindComponent( eventLine1RisingEdgeTimestamp, "EventLine1RisingEdgeTimestamp" );
		locator.bindComponent( eventLine1RisingEdgeFrameID, "EventLine1RisingEdgeFrameID" );
		locator.bindComponent( eventLine2RisingEdge, "EventLine2RisingEdge" );
		locator.bindComponent( eventLine2RisingEdgeTimestamp, "EventLine2RisingEdgeTimestamp" );
		locator.bindComponent( eventLine2RisingEdgeFrameID, "EventLine2RisingEdgeFrameID" );
		locator.bindComponent( eventLine3RisingEdge, "EventLine3RisingEdge" );
		locator.bindComponent( eventLine3RisingEdgeTimestamp, "EventLine3RisingEdgeTimestamp" );
		locator.bindComponent( eventLine3RisingEdgeFrameID, "EventLine3RisingEdgeFrameID" );
		locator.bindComponent( eventLine0FallingEdge, "EventLine0FallingEdge" );
		locator.bindComponent( eventLine0FallingEdgeTimestamp, "EventLine0FallingEdgeTimestamp" );
		locator.bindComponent( eventLine0FallingEdgeFrameID, "EventLine0FallingEdgeFrameID" );
		locator.bindComponent( eventLine1FallingEdge, "EventLine1FallingEdge" );
		locator.bindComponent( eventLine1FallingEdgeTimestamp, "EventLine1FallingEdgeTimestamp" );
		locator.bindComponent( eventLine1FallingEdgeFrameID, "EventLine1FallingEdgeFrameID" );
		locator.bindComponent( eventLine2FallingEdge, "EventLine2FallingEdge" );
		locator.bindComponent( eventLine2FallingEdgeTimestamp, "EventLine2FallingEdgeTimestamp" );
		locator.bindComponent( eventLine2FallingEdgeFrameID, "EventLine2FallingEdgeFrameID" );
		locator.bindComponent( eventLine3FallingEdge, "EventLine3FallingEdge" );
		locator.bindComponent( eventLine3FallingEdgeTimestamp, "EventLine3FallingEdgeTimestamp" );
		locator.bindComponent( eventLine3FallingEdgeFrameID, "EventLine3FallingEdgeFrameID" );
		locator.bindComponent( eventLine0AnyEdge, "EventLine0AnyEdge" );
		locator.bindComponent( eventLine0AnyEdgeTimestamp, "EventLine0AnyEdgeTimestamp" );
		locator.bindComponent( eventLine0AnyEdgeFrameID, "EventLine0AnyEdgeFrameID" );
		locator.bindComponent( eventLine1AnyEdge, "EventLine1AnyEdge" );
		locator.bindComponent( eventLine1AnyEdgeTimestamp, "EventLine1AnyEdgeTimestamp" );
		locator.bindComponent( eventLine1AnyEdgeFrameID, "EventLine1AnyEdgeFrameID" );
		locator.bindComponent( eventLine2AnyEdge, "EventLine2AnyEdge" );
		locator.bindComponent( eventLine2AnyEdgeTimestamp, "EventLine2AnyEdgeTimestamp" );
		locator.bindComponent( eventLine2AnyEdgeFrameID, "EventLine2AnyEdgeFrameID" );
		locator.bindComponent( eventLine3AnyEdge, "EventLine3AnyEdge" );
		locator.bindComponent( eventLine3AnyEdgeTimestamp, "EventLine3AnyEdgeTimestamp" );
		locator.bindComponent( eventLine3AnyEdgeFrameID, "EventLine3AnyEdgeFrameID" );
		locator.bindComponent( eventError, "EventError" );
		locator.bindComponent( eventErrorTimestamp, "EventErrorTimestamp" );
		locator.bindComponent( eventErrorFrameID, "EventErrorFrameID" );
		locator.bindComponent( eventErrorCode, "EventErrorCode" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects which Event to signal to the host application.
	///
	/// Selects which Event to signal to the host application.
	PropertyI64 eventSelector;
	/// \brief Activate or deactivate the notification to the host application of the occurrence of the selected Event.
	///
	/// Activate or deactivate the notification to the host application of the occurrence of the selected Event.
	PropertyI64 eventNotification;
	/// \brief Returns the unique Identifier of the Acquisition Trigger type of Event.
	///
	/// Returns the unique Identifier of the Acquisition Trigger type of Event.
	PropertyI64 eventAcquisitionTrigger;
	/// \brief Returns the Timestamp of the Acquisition Trigger Event.
	///
	/// Returns the Timestamp of the Acquisition Trigger Event.
	PropertyI64 eventAcquisitionTriggerTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition Trigger Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition Trigger Event.
	PropertyI64 eventAcquisitionTriggerFrameID;
	/// \brief Returns the unique Identifier of the Acquisition Start type of Event.
	///
	/// Returns the unique Identifier of the Acquisition Start type of Event.
	PropertyI64 eventAcquisitionStart;
	/// \brief Returns the Timestamp of the Acquisition Start Event.
	///
	/// Returns the Timestamp of the Acquisition Start Event.
	PropertyI64 eventAcquisitionStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition Start Event.
	PropertyI64 eventAcquisitionStartFrameID;
	/// \brief Returns the unique Identifier of the Acquisition End type of Event.
	///
	/// Returns the unique Identifier of the Acquisition End type of Event.
	PropertyI64 eventAcquisitionEnd;
	/// \brief Returns the Timestamp of the Acquisition End Event.
	///
	/// Returns the Timestamp of the Acquisition End Event.
	PropertyI64 eventAcquisitionEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition End Event.
	PropertyI64 eventAcquisitionEndFrameID;
	/// \brief Returns the unique Identifier of the Acquisition Transfer Start type of Event.
	///
	/// Returns the unique Identifier of the Acquisition Transfer Start type of Event.
	PropertyI64 eventAcquisitionTransferStart;
	/// \brief Returns the Timestamp of the Acquisition Transfer Start Event.
	///
	/// Returns the Timestamp of the Acquisition Transfer Start Event.
	PropertyI64 eventAcquisitionTransferStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition Transfer Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition Transfer Start Event.
	PropertyI64 eventAcquisitionTransferStartFrameID;
	/// \brief Returns the unique Identifier of the Acquisition Transfer End type of Event.
	///
	/// Returns the unique Identifier of the Acquisition Transfer End type of Event.
	PropertyI64 eventAcquisitionTransferEnd;
	/// \brief Returns the Timestamp of the Acquisition Transfer End Event.
	///
	/// Returns the Timestamp of the Acquisition Transfer End Event.
	PropertyI64 eventAcquisitionTransferEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition Transfer End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition Transfer End Event.
	PropertyI64 eventAcquisitionTransferEndFrameID;
	/// \brief Returns the unique Identifier of the Acquisition Error type of Event.
	///
	/// Returns the unique Identifier of the Acquisition Error type of Event.
	PropertyI64 eventAcquisitionError;
	/// \brief Returns the Timestamp of the Acquisition Error Event.
	///
	/// Returns the Timestamp of the Acquisition Error Event.
	PropertyI64 eventAcquisitionErrorTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Acquisition Error Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Acquisition Error Event.
	PropertyI64 eventAcquisitionErrorFrameID;
	/// \brief Returns the unique Identifier of the FrameTrigger type of Event.
	///
	/// Returns the unique Identifier of the FrameTrigger type of Event. It can be used to register a callback function to be notified of the event occurrence. Its value uniquely identify the type event received.
	PropertyI64 eventFrameTrigger;
	/// \brief Returns the Timestamp of the AcquisitionTrigger Event.
	///
	/// Returns the Timestamp of the AcquisitionTrigger Event. It can be used to determine precisely when the event occured.
	PropertyI64 eventFrameTriggerTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the FrameTrigger Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the FrameTrigger Event.
	PropertyI64 eventFrameTriggerFrameID;
	/// \brief Returns the unique Identifier of the Frame Start type of Event.
	///
	/// Returns the unique Identifier of the Frame Start type of Event.
	PropertyI64 eventFrameStart;
	/// \brief Returns the Timestamp of the Frame Start Event.
	///
	/// Returns the Timestamp of the Frame Start Event.
	PropertyI64 eventFrameStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame Start Event.
	PropertyI64 eventFrameStartFrameID;
	/// \brief Returns the unique Identifier of the Frame End type of Event.
	///
	/// Returns the unique Identifier of the Frame End type of Event.
	PropertyI64 eventFrameEnd;
	/// \brief Returns the Timestamp of the Frame End Event.
	///
	/// Returns the Timestamp of the Frame End Event.
	PropertyI64 eventFrameEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame End Event.
	PropertyI64 eventFrameEndFrameID;
	/// \brief Returns the unique Identifier of the Frame Burst Start type of Event.
	///
	/// Returns the unique Identifier of the Frame Burst Start type of Event.
	PropertyI64 eventFrameBurstStart;
	/// \brief Returns the Timestamp of the Frame Burst Start Event.
	///
	/// Returns the Timestamp of the Frame Burst Start Event.
	PropertyI64 eventFrameBurstStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame Burst Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame Burst Start Event.
	PropertyI64 eventFrameBurstStartFrameID;
	/// \brief Returns the unique Identifier of the Frame Burst End type of Event.
	///
	/// Returns the unique Identifier of the Frame Burst End type of Event.
	PropertyI64 eventFrameBurstEnd;
	/// \brief Returns the Timestamp of the Frame Burst End Event.
	///
	/// Returns the Timestamp of the Frame Burst End Event.
	PropertyI64 eventFrameBurstEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame Burst End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame Burst End Event.
	PropertyI64 eventFrameBurstEndFrameID;
	/// \brief Returns the unique Identifier of the Frame Transfer Start type of Event.
	///
	/// Returns the unique Identifier of the Frame Transfer Start type of Event.
	PropertyI64 eventFrameTransferStart;
	/// \brief Returns the Timestamp of the Frame Transfer Start Event.
	///
	/// Returns the Timestamp of the Frame Transfer Start Event.
	PropertyI64 eventFrameTransferStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame Transfer Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame Transfer Start Event.
	PropertyI64 eventFrameTransferStartFrameID;
	/// \brief Returns the unique Identifier of the Frame Transfer End type of Event.
	///
	/// Returns the unique Identifier of the Frame Transfer End type of Event.
	PropertyI64 eventFrameTransferEnd;
	/// \brief Returns the Timestamp of the Frame Transfer End Event.
	///
	/// Returns the Timestamp of the Frame Transfer End Event.
	PropertyI64 eventFrameTransferEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Frame Transfer End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Frame Transfer End Event.
	PropertyI64 eventFrameTransferEndFrameID;
	/// \brief Returns the unique Identifier of the Exposure Start type of Event.
	///
	/// Returns the unique Identifier of the Exposure Start type of Event.
	PropertyI64 eventExposureStart;
	/// \brief Returns the Timestamp of the Exposure Start Event.
	///
	/// Returns the Timestamp of the Exposure Start Event.
	PropertyI64 eventExposureStartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Exposure Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Exposure Start Event.
	PropertyI64 eventExposureStartFrameID;
	/// \brief Returns the unique identifier of the ExposureEnd type of Event.
	///
	/// Returns the unique identifier of the ExposureEnd type of Event. This feature can be used to register a callback function to be notified of the event occurrence. Its value uniquely identifies the type of event that will be received.
	PropertyI64 eventExposureEnd;
	/// \brief Returns the Timestamp of the ExposureEnd Event.
	///
	/// Returns the Timestamp of the ExposureEnd Event. It can be used to determine precisely when the event occured.
	PropertyI64 eventExposureEndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the ExposureEnd Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the ExposureEnd Event.
	PropertyI64 eventExposureEndFrameID;
	/// \brief Returns the unique Identifier of the Counter 1 Start type of Event.
	///
	/// Returns the unique Identifier of the Counter 1 Start type of Event.
	PropertyI64 eventCounter1Start;
	/// \brief Returns the Timestamp of the Counter 1 Start Event.
	///
	/// Returns the Timestamp of the Counter 1 Start Event.
	PropertyI64 eventCounter1StartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Counter 1 Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Counter 1 Start Event.
	PropertyI64 eventCounter1StartFrameID;
	PropertyI64 eventCounter2Start;
	PropertyI64 eventCounter2StartTimestamp;
	PropertyI64 eventCounter2StartFrameID;
	/// \brief Returns the unique Identifier of the Counter 1 End type of Event.
	///
	/// Returns the unique Identifier of the Counter 1 End type of Event.
	PropertyI64 eventCounter1End;
	/// \brief Returns the Timestamp of the Counter 1 End Event.
	///
	/// Returns the Timestamp of the Counter 1 End Event.
	PropertyI64 eventCounter1EndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Counter 1 End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Counter 1 End Event.
	PropertyI64 eventCounter1EndFrameID;
	PropertyI64 eventCounter2End;
	PropertyI64 eventCounter2EndTimestamp;
	PropertyI64 eventCounter2EndFrameID;
	/// \brief Returns the unique Identifier of the Timer 1 Start type of Event.
	///
	/// Returns the unique Identifier of the Timer 1 Start type of Event.
	PropertyI64 eventTimer1Start;
	/// \brief Returns the Timestamp of the Timer 1 Start Event.
	///
	/// Returns the Timestamp of the Timer 1 Start Event.
	PropertyI64 eventTimer1StartTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Timer 1 Start Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Timer 1 Start Event.
	PropertyI64 eventTimer1StartFrameID;
	PropertyI64 eventTimer2Start;
	PropertyI64 eventTimer2StartTimestamp;
	PropertyI64 eventTimer2StartFrameID;
	/// \brief Returns the unique Identifier of the Timer 1 End type of Event.
	///
	/// Returns the unique Identifier of the Timer 1 End type of Event.
	PropertyI64 eventTimer1End;
	/// \brief Returns the Timestamp of the Timer 1 End Event.
	///
	/// Returns the Timestamp of the Timer 1 End Event.
	PropertyI64 eventTimer1EndTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Timer 1 End Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Timer 1 End Event.
	PropertyI64 eventTimer1EndFrameID;
	PropertyI64 eventTimer2End;
	PropertyI64 eventTimer2EndTimestamp;
	PropertyI64 eventTimer2EndFrameID;
	/// \brief Returns the unique Identifier of the Line 0 Rising Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 0 Rising Edge type of Event.
	PropertyI64 eventLine0RisingEdge;
	/// \brief Returns the Timestamp of the Line 0 Rising Edge Event.
	///
	/// Returns the Timestamp of the Line 0 Rising Edge Event.
	PropertyI64 eventLine0RisingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 0 Rising Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 0 Rising Edge Event.
	PropertyI64 eventLine0RisingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 1 Rising Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 1 Rising Edge type of Event.
	PropertyI64 eventLine1RisingEdge;
	/// \brief Returns the Timestamp of the Line 1 Rising Edge Event.
	///
	/// Returns the Timestamp of the Line 1 Rising Edge Event.
	PropertyI64 eventLine1RisingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 1 Rising Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 1 Rising Edge Event.
	PropertyI64 eventLine1RisingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 2 Rising Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 2 Rising Edge type of Event.
	PropertyI64 eventLine2RisingEdge;
	/// \brief Returns the Timestamp of the Line 2 Rising Edge Event.
	///
	/// Returns the Timestamp of the Line 2 Rising Edge Event.
	PropertyI64 eventLine2RisingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 2 Rising Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 2 Rising Edge Event.
	PropertyI64 eventLine2RisingEdgeFrameID;
	PropertyI64 eventLine3RisingEdge;
	PropertyI64 eventLine3RisingEdgeTimestamp;
	PropertyI64 eventLine3RisingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 0 Falling Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 0 Falling Edge type of Event.
	PropertyI64 eventLine0FallingEdge;
	/// \brief Returns the Timestamp of the Line 0 Falling Edge Event.
	///
	/// Returns the Timestamp of the Line 0 Falling Edge Event.
	PropertyI64 eventLine0FallingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 0 Falling Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 0 Falling Edge Event.
	PropertyI64 eventLine0FallingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 1 Falling Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 1 Falling Edge type of Event.
	PropertyI64 eventLine1FallingEdge;
	/// \brief Returns the Timestamp of the Line 1 Falling Edge Event.
	///
	/// Returns the Timestamp of the Line 1 Falling Edge Event.
	PropertyI64 eventLine1FallingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 1 Falling Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 1 Falling Edge Event.
	PropertyI64 eventLine1FallingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 2 Falling Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 2 Falling Edge type of Event.
	PropertyI64 eventLine2FallingEdge;
	/// \brief Returns the Timestamp of the Line 2 Falling Edge Event.
	///
	/// Returns the Timestamp of the Line 2 Falling Edge Event.
	PropertyI64 eventLine2FallingEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 2 Falling Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 2 Falling Edge Event.
	PropertyI64 eventLine2FallingEdgeFrameID;
	PropertyI64 eventLine3FallingEdge;
	PropertyI64 eventLine3FallingEdgeTimestamp;
	PropertyI64 eventLine3FallingEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 0 Any Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 0 Any Edge type of Event.
	PropertyI64 eventLine0AnyEdge;
	/// \brief Returns the Timestamp of the Line 0 Any Edge Event.
	///
	/// Returns the Timestamp of the Line 0 Any Edge Event.
	PropertyI64 eventLine0AnyEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 0 Any Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 0 Any Edge Event.
	PropertyI64 eventLine0AnyEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 1 Any Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 1 Any Edge type of Event.
	PropertyI64 eventLine1AnyEdge;
	/// \brief Returns the Timestamp of the Line 1 Any Edge Event.
	///
	/// Returns the Timestamp of the Line 1 Any Edge Event.
	PropertyI64 eventLine1AnyEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 1 Any Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 1 Any Edge Event.
	PropertyI64 eventLine1AnyEdgeFrameID;
	/// \brief Returns the unique Identifier of the Line 2 Any Edge type of Event.
	///
	/// Returns the unique Identifier of the Line 2 Any Edge type of Event.
	PropertyI64 eventLine2AnyEdge;
	/// \brief Returns the Timestamp of the Line 2 Any Edge Event.
	///
	/// Returns the Timestamp of the Line 2 Any Edge Event.
	PropertyI64 eventLine2AnyEdgeTimestamp;
	/// \brief Returns the unique Identifier of the Frame (or image) that generated the Line 2 Any Edge Event.
	///
	/// Returns the unique Identifier of the Frame (or image) that generated the Line 2 Any Edge Event.
	PropertyI64 eventLine2AnyEdgeFrameID;
	PropertyI64 eventLine3AnyEdge;
	PropertyI64 eventLine3AnyEdgeTimestamp;
	PropertyI64 eventLine3AnyEdgeFrameID;
	/// \brief Returns the unique identifier of the Error type of Event.
	///
	/// Returns the unique identifier of the Error type of Event. It can be used to register a callback function to be notified of the Error event occurrence. Its value uniquely identify that the event received was an Error.
	PropertyI64 eventError;
	/// \brief Returns the Timestamp of the Error Event.
	///
	/// Returns the Timestamp of the Error Event. It can be used to determine when the event occured.
	PropertyI64 eventErrorTimestamp;
	/// \brief If applicable, returns the unique Identifier of the Frame (or image) that generated the Error Event.
	///
	/// If applicable, returns the unique Identifier of the Frame (or image) that generated the Error Event.
	PropertyI64 eventErrorFrameID;
	/// \brief Returns an error code for the error(s) that happened.
	///
	/// Returns an error code for the error(s) that happened.
	PropertyI64 eventErrorCode;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getEventSelector( void ) const { return eventSelector; }
	PropertyI64 getEventNotification( void ) const { return eventNotification; }
	PropertyI64 getEventAcquisitionTrigger( void ) const { return eventAcquisitionTrigger; }
	PropertyI64 getEventAcquisitionTriggerTimestamp( void ) const { return eventAcquisitionTriggerTimestamp; }
	PropertyI64 getEventAcquisitionTriggerFrameID( void ) const { return eventAcquisitionTriggerFrameID; }
	PropertyI64 getEventAcquisitionStart( void ) const { return eventAcquisitionStart; }
	PropertyI64 getEventAcquisitionStartTimestamp( void ) const { return eventAcquisitionStartTimestamp; }
	PropertyI64 getEventAcquisitionStartFrameID( void ) const { return eventAcquisitionStartFrameID; }
	PropertyI64 getEventAcquisitionEnd( void ) const { return eventAcquisitionEnd; }
	PropertyI64 getEventAcquisitionEndTimestamp( void ) const { return eventAcquisitionEndTimestamp; }
	PropertyI64 getEventAcquisitionEndFrameID( void ) const { return eventAcquisitionEndFrameID; }
	PropertyI64 getEventAcquisitionTransferStart( void ) const { return eventAcquisitionTransferStart; }
	PropertyI64 getEventAcquisitionTransferStartTimestamp( void ) const { return eventAcquisitionTransferStartTimestamp; }
	PropertyI64 getEventAcquisitionTransferStartFrameID( void ) const { return eventAcquisitionTransferStartFrameID; }
	PropertyI64 getEventAcquisitionTransferEnd( void ) const { return eventAcquisitionTransferEnd; }
	PropertyI64 getEventAcquisitionTransferEndTimestamp( void ) const { return eventAcquisitionTransferEndTimestamp; }
	PropertyI64 getEventAcquisitionTransferEndFrameID( void ) const { return eventAcquisitionTransferEndFrameID; }
	PropertyI64 getEventAcquisitionError( void ) const { return eventAcquisitionError; }
	PropertyI64 getEventAcquisitionErrorTimestamp( void ) const { return eventAcquisitionErrorTimestamp; }
	PropertyI64 getEventAcquisitionErrorFrameID( void ) const { return eventAcquisitionErrorFrameID; }
	PropertyI64 getEventFrameTrigger( void ) const { return eventFrameTrigger; }
	PropertyI64 getEventFrameTriggerTimestamp( void ) const { return eventFrameTriggerTimestamp; }
	PropertyI64 getEventFrameTriggerFrameID( void ) const { return eventFrameTriggerFrameID; }
	PropertyI64 getEventFrameStart( void ) const { return eventFrameStart; }
	PropertyI64 getEventFrameStartTimestamp( void ) const { return eventFrameStartTimestamp; }
	PropertyI64 getEventFrameStartFrameID( void ) const { return eventFrameStartFrameID; }
	PropertyI64 getEventFrameEnd( void ) const { return eventFrameEnd; }
	PropertyI64 getEventFrameEndTimestamp( void ) const { return eventFrameEndTimestamp; }
	PropertyI64 getEventFrameEndFrameID( void ) const { return eventFrameEndFrameID; }
	PropertyI64 getEventFrameBurstStart( void ) const { return eventFrameBurstStart; }
	PropertyI64 getEventFrameBurstStartTimestamp( void ) const { return eventFrameBurstStartTimestamp; }
	PropertyI64 getEventFrameBurstStartFrameID( void ) const { return eventFrameBurstStartFrameID; }
	PropertyI64 getEventFrameBurstEnd( void ) const { return eventFrameBurstEnd; }
	PropertyI64 getEventFrameBurstEndTimestamp( void ) const { return eventFrameBurstEndTimestamp; }
	PropertyI64 getEventFrameBurstEndFrameID( void ) const { return eventFrameBurstEndFrameID; }
	PropertyI64 getEventFrameTransferStart( void ) const { return eventFrameTransferStart; }
	PropertyI64 getEventFrameTransferStartTimestamp( void ) const { return eventFrameTransferStartTimestamp; }
	PropertyI64 getEventFrameTransferStartFrameID( void ) const { return eventFrameTransferStartFrameID; }
	PropertyI64 getEventFrameTransferEnd( void ) const { return eventFrameTransferEnd; }
	PropertyI64 getEventFrameTransferEndTimestamp( void ) const { return eventFrameTransferEndTimestamp; }
	PropertyI64 getEventFrameTransferEndFrameID( void ) const { return eventFrameTransferEndFrameID; }
	PropertyI64 getEventExposureStart( void ) const { return eventExposureStart; }
	PropertyI64 getEventExposureStartTimestamp( void ) const { return eventExposureStartTimestamp; }
	PropertyI64 getEventExposureStartFrameID( void ) const { return eventExposureStartFrameID; }
	PropertyI64 getEventExposureEnd( void ) const { return eventExposureEnd; }
	PropertyI64 getEventExposureEndTimestamp( void ) const { return eventExposureEndTimestamp; }
	PropertyI64 getEventExposureEndFrameID( void ) const { return eventExposureEndFrameID; }
	PropertyI64 getEventCounter1Start( void ) const { return eventCounter1Start; }
	PropertyI64 getEventCounter1StartTimestamp( void ) const { return eventCounter1StartTimestamp; }
	PropertyI64 getEventCounter1StartFrameID( void ) const { return eventCounter1StartFrameID; }
	PropertyI64 getEventCounter2Start( void ) const { return eventCounter2Start; }
	PropertyI64 getEventCounter2StartTimestamp( void ) const { return eventCounter2StartTimestamp; }
	PropertyI64 getEventCounter2StartFrameID( void ) const { return eventCounter2StartFrameID; }
	PropertyI64 getEventCounter1End( void ) const { return eventCounter1End; }
	PropertyI64 getEventCounter1EndTimestamp( void ) const { return eventCounter1EndTimestamp; }
	PropertyI64 getEventCounter1EndFrameID( void ) const { return eventCounter1EndFrameID; }
	PropertyI64 getEventCounter2End( void ) const { return eventCounter2End; }
	PropertyI64 getEventCounter2EndTimestamp( void ) const { return eventCounter2EndTimestamp; }
	PropertyI64 getEventCounter2EndFrameID( void ) const { return eventCounter2EndFrameID; }
	PropertyI64 getEventTimer1Start( void ) const { return eventTimer1Start; }
	PropertyI64 getEventTimer1StartTimestamp( void ) const { return eventTimer1StartTimestamp; }
	PropertyI64 getEventTimer1StartFrameID( void ) const { return eventTimer1StartFrameID; }
	PropertyI64 getEventTimer2Start( void ) const { return eventTimer2Start; }
	PropertyI64 getEventTimer2StartTimestamp( void ) const { return eventTimer2StartTimestamp; }
	PropertyI64 getEventTimer2StartFrameID( void ) const { return eventTimer2StartFrameID; }
	PropertyI64 getEventTimer1End( void ) const { return eventTimer1End; }
	PropertyI64 getEventTimer1EndTimestamp( void ) const { return eventTimer1EndTimestamp; }
	PropertyI64 getEventTimer1EndFrameID( void ) const { return eventTimer1EndFrameID; }
	PropertyI64 getEventTimer2End( void ) const { return eventTimer2End; }
	PropertyI64 getEventTimer2EndTimestamp( void ) const { return eventTimer2EndTimestamp; }
	PropertyI64 getEventTimer2EndFrameID( void ) const { return eventTimer2EndFrameID; }
	PropertyI64 getEventLine0RisingEdge( void ) const { return eventLine0RisingEdge; }
	PropertyI64 getEventLine0RisingEdgeTimestamp( void ) const { return eventLine0RisingEdgeTimestamp; }
	PropertyI64 getEventLine0RisingEdgeFrameID( void ) const { return eventLine0RisingEdgeFrameID; }
	PropertyI64 getEventLine1RisingEdge( void ) const { return eventLine1RisingEdge; }
	PropertyI64 getEventLine1RisingEdgeTimestamp( void ) const { return eventLine1RisingEdgeTimestamp; }
	PropertyI64 getEventLine1RisingEdgeFrameID( void ) const { return eventLine1RisingEdgeFrameID; }
	PropertyI64 getEventLine2RisingEdge( void ) const { return eventLine2RisingEdge; }
	PropertyI64 getEventLine2RisingEdgeTimestamp( void ) const { return eventLine2RisingEdgeTimestamp; }
	PropertyI64 getEventLine2RisingEdgeFrameID( void ) const { return eventLine2RisingEdgeFrameID; }
	PropertyI64 getEventLine3RisingEdge( void ) const { return eventLine3RisingEdge; }
	PropertyI64 getEventLine3RisingEdgeTimestamp( void ) const { return eventLine3RisingEdgeTimestamp; }
	PropertyI64 getEventLine3RisingEdgeFrameID( void ) const { return eventLine3RisingEdgeFrameID; }
	PropertyI64 getEventLine0FallingEdge( void ) const { return eventLine0FallingEdge; }
	PropertyI64 getEventLine0FallingEdgeTimestamp( void ) const { return eventLine0FallingEdgeTimestamp; }
	PropertyI64 getEventLine0FallingEdgeFrameID( void ) const { return eventLine0FallingEdgeFrameID; }
	PropertyI64 getEventLine1FallingEdge( void ) const { return eventLine1FallingEdge; }
	PropertyI64 getEventLine1FallingEdgeTimestamp( void ) const { return eventLine1FallingEdgeTimestamp; }
	PropertyI64 getEventLine1FallingEdgeFrameID( void ) const { return eventLine1FallingEdgeFrameID; }
	PropertyI64 getEventLine2FallingEdge( void ) const { return eventLine2FallingEdge; }
	PropertyI64 getEventLine2FallingEdgeTimestamp( void ) const { return eventLine2FallingEdgeTimestamp; }
	PropertyI64 getEventLine2FallingEdgeFrameID( void ) const { return eventLine2FallingEdgeFrameID; }
	PropertyI64 getEventLine3FallingEdge( void ) const { return eventLine3FallingEdge; }
	PropertyI64 getEventLine3FallingEdgeTimestamp( void ) const { return eventLine3FallingEdgeTimestamp; }
	PropertyI64 getEventLine3FallingEdgeFrameID( void ) const { return eventLine3FallingEdgeFrameID; }
	PropertyI64 getEventLine0AnyEdge( void ) const { return eventLine0AnyEdge; }
	PropertyI64 getEventLine0AnyEdgeTimestamp( void ) const { return eventLine0AnyEdgeTimestamp; }
	PropertyI64 getEventLine0AnyEdgeFrameID( void ) const { return eventLine0AnyEdgeFrameID; }
	PropertyI64 getEventLine1AnyEdge( void ) const { return eventLine1AnyEdge; }
	PropertyI64 getEventLine1AnyEdgeTimestamp( void ) const { return eventLine1AnyEdgeTimestamp; }
	PropertyI64 getEventLine1AnyEdgeFrameID( void ) const { return eventLine1AnyEdgeFrameID; }
	PropertyI64 getEventLine2AnyEdge( void ) const { return eventLine2AnyEdge; }
	PropertyI64 getEventLine2AnyEdgeTimestamp( void ) const { return eventLine2AnyEdgeTimestamp; }
	PropertyI64 getEventLine2AnyEdgeFrameID( void ) const { return eventLine2AnyEdgeFrameID; }
	PropertyI64 getEventLine3AnyEdge( void ) const { return eventLine3AnyEdge; }
	PropertyI64 getEventLine3AnyEdgeTimestamp( void ) const { return eventLine3AnyEdgeTimestamp; }
	PropertyI64 getEventLine3AnyEdgeFrameID( void ) const { return eventLine3AnyEdgeFrameID; }
	PropertyI64 getEventError( void ) const { return eventError; }
	PropertyI64 getEventErrorTimestamp( void ) const { return eventErrorTimestamp; }
	PropertyI64 getEventErrorFrameID( void ) const { return eventErrorFrameID; }
	PropertyI64 getEventErrorCode( void ) const { return eventErrorCode; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the Analog control features.
///
/// Category that contains the Analog control features.
class AnalogControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::AnalogControl</b> object.
	explicit AnalogControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			gainSelector(),
			gain(),
			gainRaw(),
			gainAbs(),
			gainAuto(),
			gainAutoBalance(),
			blackLevelSelector(),
			blackLevel(),
			blackLevelRaw(),
			blackLevelAbs(),
			blackLevelAuto(),
			blackLevelAutoBalance(),
			whiteClipSelector(),
			whiteClip(),
			whiteClipRaw(),
			whiteClipAbs(),
			balanceRatioSelector(),
			balanceRatio(),
			balanceRatioAbs(),
			balanceWhiteAuto(),
			gamma(),
			mvGainAutoDelayImages(),
			mvGainAutoUpperLimit(),
			mvGainAutoLowerLimit(),
			mvGainAutoSpeed(),
			mvGainAutoAverageGrey(),
			mvGainAutoHighlightAOI(),
			mvGainAutoAOIMode(),
			mvGainAutoOffsetX(),
			mvGainAutoOffsetY(),
			mvGainAutoWidth(),
			mvGainAutoHeight(),
			mvGainAutoMode(),
			mvBalanceWhiteAutoAOIMode(),
			mvBalanceWhiteAutoOffsetX(),
			mvBalanceWhiteAutoOffsetY(),
			mvBalanceWhiteAutoWidth(),
			mvBalanceWhiteAutoHeight(),
			mvVCAL(),
			mvVBLACK(),
			mvVOFFSET(),
			mvLowLight(),
			mvADCGain(),
			mvVRamp(),
			mvDigitalGainOffset(),
			mvSaveCalibrationData()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( gainSelector, "GainSelector" );
		locator.bindComponent( gain, "Gain" );
		locator.bindComponent( gainRaw, "GainRaw" );
		locator.bindComponent( gainAbs, "GainAbs" );
		locator.bindComponent( gainAuto, "GainAuto" );
		locator.bindComponent( gainAutoBalance, "GainAutoBalance" );
		locator.bindComponent( blackLevelSelector, "BlackLevelSelector" );
		locator.bindComponent( blackLevel, "BlackLevel" );
		locator.bindComponent( blackLevelRaw, "BlackLevelRaw" );
		locator.bindComponent( blackLevelAbs, "BlackLevelAbs" );
		locator.bindComponent( blackLevelAuto, "BlackLevelAuto" );
		locator.bindComponent( blackLevelAutoBalance, "BlackLevelAutoBalance" );
		locator.bindComponent( whiteClipSelector, "WhiteClipSelector" );
		locator.bindComponent( whiteClip, "WhiteClip" );
		locator.bindComponent( whiteClipRaw, "WhiteClipRaw" );
		locator.bindComponent( whiteClipAbs, "WhiteClipAbs" );
		locator.bindComponent( balanceRatioSelector, "BalanceRatioSelector" );
		locator.bindComponent( balanceRatio, "BalanceRatio" );
		locator.bindComponent( balanceRatioAbs, "BalanceRatioAbs" );
		locator.bindComponent( balanceWhiteAuto, "BalanceWhiteAuto" );
		locator.bindComponent( gamma, "Gamma" );
		locator.bindComponent( mvGainAutoDelayImages, "mvGainAutoDelayImages" );
		locator.bindComponent( mvGainAutoUpperLimit, "mvGainAutoUpperLimit" );
		locator.bindComponent( mvGainAutoLowerLimit, "mvGainAutoLowerLimit" );
		locator.bindComponent( mvGainAutoSpeed, "mvGainAutoSpeed" );
		locator.bindComponent( mvGainAutoAverageGrey, "mvGainAutoAverageGrey" );
		locator.bindComponent( mvGainAutoHighlightAOI, "mvGainAutoHighlightAOI" );
		locator.bindComponent( mvGainAutoAOIMode, "mvGainAutoAOIMode" );
		locator.bindComponent( mvGainAutoOffsetX, "mvGainAutoOffsetX" );
		locator.bindComponent( mvGainAutoOffsetY, "mvGainAutoOffsetY" );
		locator.bindComponent( mvGainAutoWidth, "mvGainAutoWidth" );
		locator.bindComponent( mvGainAutoHeight, "mvGainAutoHeight" );
		locator.bindComponent( mvGainAutoMode, "mvGainAutoMode" );
		locator.bindComponent( mvBalanceWhiteAutoAOIMode, "mvBalanceWhiteAutoAOIMode" );
		locator.bindComponent( mvBalanceWhiteAutoOffsetX, "mvBalanceWhiteAutoOffsetX" );
		locator.bindComponent( mvBalanceWhiteAutoOffsetY, "mvBalanceWhiteAutoOffsetY" );
		locator.bindComponent( mvBalanceWhiteAutoWidth, "mvBalanceWhiteAutoWidth" );
		locator.bindComponent( mvBalanceWhiteAutoHeight, "mvBalanceWhiteAutoHeight" );
		locator.bindComponent( mvVCAL, "mvVCAL" );
		locator.bindComponent( mvVBLACK, "mvVBLACK" );
		locator.bindComponent( mvVOFFSET, "mvVOFFSET" );
		locator.bindComponent( mvLowLight, "mvLowLight" );
		locator.bindComponent( mvADCGain, "mvADCGain" );
		locator.bindComponent( mvVRamp, "mvVRamp" );
		locator.bindComponent( mvDigitalGainOffset, "mvDigitalGainOffset" );
		locator.bindComponent( mvSaveCalibrationData, "mvSaveCalibrationData@i" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects which Gain is controlled by the various Gain features.
	///
	/// Selects which Gain is controlled by the various Gain features.
	PropertyI64 gainSelector;
	/// \brief Controls the selected gain as an absolute physical value.
	///
	/// Controls the selected gain as an absolute physical value. This is an amplification factor applied to the video signal.
	PropertyF gain;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the selected gain as a raw integer value. This is an amplification factor applied to the video signal.
	PropertyI64 gainRaw;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. This feature controls the selected gain as an absolute physical value. This is an amplification factor applied to the video signal.
	PropertyF gainAbs;
	/// \brief Sets the automatic gain control (AGC) mode.
	///
	/// Sets the automatic gain control (AGC) mode. The exact algorithm used to implement AGC is device-specific.
	PropertyI64 gainAuto;
	/// \brief Sets the mode for automatic gain balancing between the sensor color channels or taps.
	///
	/// Sets the mode for automatic gain balancing between the sensor color channels or taps. The gain coefficients of each channel or tap are adjusted so they are matched.
	PropertyI64 gainAutoBalance;
	/// \brief Selects which Black Level is controlled by the various Black Level features.
	///
	/// Selects which Black Level is controlled by the various Black Level features.
	PropertyI64 blackLevelSelector;
	/// \brief Controls the analog black level as an absolute physical value.
	///
	/// Controls the analog black level as an absolute physical value. This represents a DC offset applied to the video signal.
	PropertyF blackLevel;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the analog black level as a raw integer value. This represents a DC offset applied to the video signal.
	PropertyI64 blackLevelRaw;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. This feature controls the analog black level as an absolute physical value. This represents a DC offset applied to the video signal.
	PropertyF blackLevelAbs;
	/// \brief Controls the mode for automatic black level adjustment.
	///
	/// Controls the mode for automatic black level adjustment. The exact algorithm used to implement this adjustment is device-specific.
	PropertyI64 blackLevelAuto;
	/// \brief Controls the mode for automatic black level balancing between the sensor color channels or taps.
	///
	/// Controls the mode for automatic black level balancing between the sensor color channels or taps. The black level coefficients of each channel are adjusted so they are matched.
	PropertyI64 blackLevelAutoBalance;
	/// \brief Selects which White Clip to control.
	///
	/// Selects which White Clip to control.
	PropertyI64 whiteClipSelector;
	/// \brief Controls the maximal intensity taken by the video signal before being clipped as an absolute physical value.
	///
	/// Controls the maximal intensity taken by the video signal before being clipped as an absolute physical value. The video signal will never exceed the white clipping point: it will saturate at that level.
	PropertyF whiteClip;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. Controls the maximal intensity taken by the video signal before being clipped as a raw integer value. The video signal will never exceed the white clipping point: it will saturate at that level.
	PropertyI64 whiteClipRaw;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. Controls the maximal intensity taken by the video signal before being clipped as an absolute physical value. The video signal will never exceed the white clipping point: it will saturate at that level.
	PropertyF whiteClipAbs;
	/// \brief Selects which Balance ratio to control.
	///
	/// Selects which Balance ratio to control.
	PropertyI64 balanceRatioSelector;
	/// \brief Controls ratio of the selected color component to a reference color component.
	///
	/// Controls ratio of the selected color component to a reference color component. It is used for white balancing.
	PropertyF balanceRatio;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. It controls the ratio of the selected color component to a reference color component. This feature is used for white balancing.
	PropertyF balanceRatioAbs;
	/// \brief Controls the mode for automatic white balancing between the color channels.
	///
	/// Controls the mode for automatic white balancing between the color channels. The white balancing ratios are automatically adjusted.
	PropertyI64 balanceWhiteAuto;
	/// \brief Controls the gamma correction of pixel intensity.
	///
	/// Controls the gamma correction of pixel intensity. This is typically used to compensate for non-linearity of the display system (such as CRT).
	PropertyF gamma;
	/// \brief The number of frames that the AEC must skip before updating the exposure register
	///
	/// The number of frames that the AEC must skip before updating the exposure register
	PropertyI64 mvGainAutoDelayImages;
	/// \brief The upper limit of the gain in auto gain mode
	///
	/// The upper limit of the gain in auto gain mode
	PropertyF mvGainAutoUpperLimit;
	/// \brief The lower limit of the gain in auto gain mode
	///
	/// The lower limit of the gain in auto gain mode
	PropertyF mvGainAutoLowerLimit;
	/// \brief Determines the increment or decrement size of gain value from frame to frame.
	///
	/// Determines the increment or decrement size of gain value from frame to frame.
	PropertyI64 mvGainAutoSpeed;
	/// \brief Common desired average grey value (in percent) used for Auto Gain Control(AGC) and Auto Exposure Control(AEC).
	///
	/// Common desired average grey value (in percent) used for Auto Gain Control(AGC) and Auto Exposure Control(AEC).
	PropertyI64 mvGainAutoAverageGrey;
	/// \brief Highlight auto control AOI to check AOI settings. Switch off for normal operation.
	///
	/// Highlight auto control AOI to check AOI settings. Switch off for normal operation.
	PropertyI64 mvGainAutoHighlightAOI;
	/// \brief Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balancing.
	///
	/// Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balancing.
	PropertyI64 mvGainAutoAOIMode;
	/// \brief Common AOI X-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI X-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvGainAutoOffsetX;
	/// \brief Common AOI Y-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Y-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvGainAutoOffsetY;
	/// \brief Common AOI Width used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Width used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvGainAutoWidth;
	/// \brief Common AOI Height used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Height used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvGainAutoHeight;
	/// \brief Selects the common auto mode for gain and exposure.
	///
	/// Selects the common auto mode for gain and exposure.
	PropertyI64 mvGainAutoMode;
	/// \brief Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AutoControl AOI used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvBalanceWhiteAutoAOIMode;
	/// \brief Common AOI X-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI XOffset used for auto gain control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvBalanceWhiteAutoOffsetX;
	/// \brief Common AOI Y-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Y-Offset used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvBalanceWhiteAutoOffsetY;
	/// \brief Common AOI Width used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Width used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvBalanceWhiteAutoWidth;
	/// \brief Common AOI Height used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	///
	/// Common AOI Height used for Auto Gain Control(AGC), Auto Exposure Control(AEC) and Auto White Balance(AWB).
	PropertyI64 mvBalanceWhiteAutoHeight;
	/// \brief Sets the voltage in millivolt.
	///
	/// Sets the voltage in millivolt.
	PropertyI64 mvVCAL;
	/// \brief Sets the voltage in millivolt.
	///
	/// Sets the voltage in millivolt.
	PropertyI64 mvVBLACK;
	/// \brief Sets the voltage in millivolt.
	///
	/// Sets the voltage in millivolt.
	PropertyI64 mvVOFFSET;
	/// \brief Makes the image brighter.
	///
	/// Makes the image brighter.
	PropertyI64 mvLowLight;
	/// \brief Adapt gain. Gain value of the sensor may differ from sensor to sensor.
	///
	/// Adapt gain. Gain value of the sensor may differ from sensor to sensor.
	PropertyI64 mvADCGain;
	/// \brief Adjusting this value will result in better column CDS (correlated double sampling) which will remove the column FPN from the image.
	///
	/// Adjusting this value will result in better column CDS (correlated double sampling) which will remove the column FPN from the image.
	PropertyI64 mvVRamp;
	/// \brief Used for fine tuning of the brightness of the sensor.
	///
	/// Used for fine tuning of the brightness of the sensor.
	PropertyI64 mvDigitalGainOffset;
	Method mvSaveCalibrationData;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getGainSelector( void ) const { return gainSelector; }
	PropertyF getGain( void ) const { return gain; }
	PropertyI64 getGainRaw( void ) const { return gainRaw; }
	PropertyF getGainAbs( void ) const { return gainAbs; }
	PropertyI64 getGainAuto( void ) const { return gainAuto; }
	PropertyI64 getGainAutoBalance( void ) const { return gainAutoBalance; }
	PropertyI64 getBlackLevelSelector( void ) const { return blackLevelSelector; }
	PropertyF getBlackLevel( void ) const { return blackLevel; }
	PropertyI64 getBlackLevelRaw( void ) const { return blackLevelRaw; }
	PropertyF getBlackLevelAbs( void ) const { return blackLevelAbs; }
	PropertyI64 getBlackLevelAuto( void ) const { return blackLevelAuto; }
	PropertyI64 getBlackLevelAutoBalance( void ) const { return blackLevelAutoBalance; }
	PropertyI64 getWhiteClipSelector( void ) const { return whiteClipSelector; }
	PropertyF getWhiteClip( void ) const { return whiteClip; }
	PropertyI64 getWhiteClipRaw( void ) const { return whiteClipRaw; }
	PropertyF getWhiteClipAbs( void ) const { return whiteClipAbs; }
	PropertyI64 getBalanceRatioSelector( void ) const { return balanceRatioSelector; }
	PropertyF getBalanceRatio( void ) const { return balanceRatio; }
	PropertyF getBalanceRatioAbs( void ) const { return balanceRatioAbs; }
	PropertyI64 getBalanceWhiteAuto( void ) const { return balanceWhiteAuto; }
	PropertyF getGamma( void ) const { return gamma; }
	PropertyI64 getmvGainAutoDelayImages( void ) const { return mvGainAutoDelayImages; }
	PropertyF getmvGainAutoUpperLimit( void ) const { return mvGainAutoUpperLimit; }
	PropertyF getmvGainAutoLowerLimit( void ) const { return mvGainAutoLowerLimit; }
	PropertyI64 getmvGainAutoSpeed( void ) const { return mvGainAutoSpeed; }
	PropertyI64 getmvGainAutoAverageGrey( void ) const { return mvGainAutoAverageGrey; }
	PropertyI64 getmvGainAutoHighlightAOI( void ) const { return mvGainAutoHighlightAOI; }
	PropertyI64 getmvGainAutoAOIMode( void ) const { return mvGainAutoAOIMode; }
	PropertyI64 getmvGainAutoOffsetX( void ) const { return mvGainAutoOffsetX; }
	PropertyI64 getmvGainAutoOffsetY( void ) const { return mvGainAutoOffsetY; }
	PropertyI64 getmvGainAutoWidth( void ) const { return mvGainAutoWidth; }
	PropertyI64 getmvGainAutoHeight( void ) const { return mvGainAutoHeight; }
	PropertyI64 getmvGainAutoMode( void ) const { return mvGainAutoMode; }
	PropertyI64 getmvBalanceWhiteAutoAOIMode( void ) const { return mvBalanceWhiteAutoAOIMode; }
	PropertyI64 getmvBalanceWhiteAutoOffsetX( void ) const { return mvBalanceWhiteAutoOffsetX; }
	PropertyI64 getmvBalanceWhiteAutoOffsetY( void ) const { return mvBalanceWhiteAutoOffsetY; }
	PropertyI64 getmvBalanceWhiteAutoWidth( void ) const { return mvBalanceWhiteAutoWidth; }
	PropertyI64 getmvBalanceWhiteAutoHeight( void ) const { return mvBalanceWhiteAutoHeight; }
	PropertyI64 getmvVCAL( void ) const { return mvVCAL; }
	PropertyI64 getmvVBLACK( void ) const { return mvVBLACK; }
	PropertyI64 getmvVOFFSET( void ) const { return mvVOFFSET; }
	PropertyI64 getmvLowLight( void ) const { return mvLowLight; }
	PropertyI64 getmvADCGain( void ) const { return mvADCGain; }
	PropertyI64 getmvVRamp( void ) const { return mvVRamp; }
	PropertyI64 getmvDigitalGainOffset( void ) const { return mvDigitalGainOffset; }
	Method getmvSaveCalibrationData( void ) const { return mvSaveCalibrationData; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that includes the LUT control features.
///
/// Category that includes the LUT control features.
class LUTControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::LUTControl</b> object.
	explicit LUTControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			LUTSelector(),
			LUTEnable(),
			LUTIndex(),
			LUTValue(),
			LUTValueAll()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( LUTSelector, "LUTSelector" );
		locator.bindComponent( LUTEnable, "LUTEnable" );
		locator.bindComponent( LUTIndex, "LUTIndex" );
		locator.bindComponent( LUTValue, "LUTValue" );
		locator.bindComponent( LUTValueAll, "LUTValueAll" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects which LUT to control.
	///
	/// Selects which LUT to control.
	PropertyI64 LUTSelector;
	/// \brief Activates the selected LUT.
	///
	/// Activates the selected LUT.
	PropertyIBoolean LUTEnable;
	/// \brief Control the index (offset) of the coefficient to access in the selected LUT.
	///
	/// Control the index (offset) of the coefficient to access in the selected LUT.
	PropertyI64 LUTIndex;
	/// \brief Returns the Value at entry LUTIndex of the LUT selected by LUTSelector.
	///
	/// Returns the Value at entry LUTIndex of the LUT selected by LUTSelector.
	PropertyI64 LUTValue;
	/// \brief Accesses all the LUT coefficients in a single access without using individual LUTIndex.
	///
	/// Accesses all the LUT coefficients in a single access without using individual LUTIndex.
	PropertyS LUTValueAll;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getLUTSelector( void ) const { return LUTSelector; }
	PropertyIBoolean getLUTEnable( void ) const { return LUTEnable; }
	PropertyI64 getLUTIndex( void ) const { return LUTIndex; }
	PropertyI64 getLUTValue( void ) const { return LUTValue; }
	PropertyS getLUTValueAll( void ) const { return LUTValueAll; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the transport Layer control features.
///
/// Category that contains the transport Layer control features.
class TransportLayerControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::TransportLayerControl</b> object.
	explicit TransportLayerControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			payloadSize(),
			gevVersionMajor(),
			gevVersionMinor(),
			gevDeviceModeIsBigEndian(),
			gevDeviceClass(),
			gevDeviceModeCharacterSet(),
			gevInterfaceSelector(),
			gevMACAddress(),
			gevSupportedOptionSelector(),
			gevSupportedOption(),
			gevSupportedIPConfigurationLLA(),
			gevSupportedIPConfigurationDHCP(),
			gevSupportedIPConfigurationPersistentIP(),
			gevCurrentIPConfiguration(),
			gevCurrentIPConfigurationLLA(),
			gevCurrentIPConfigurationDHCP(),
			gevCurrentIPConfigurationPersistentIP(),
			gevCurrentIPAddress(),
			gevCurrentSubnetMask(),
			gevCurrentDefaultGateway(),
			gevIPConfigurationStatus(),
			gevFirstURL(),
			gevSecondURL(),
			gevNumberOfInterfaces(),
			gevPersistentIPAddress(),
			gevPersistentSubnetMask(),
			gevPersistentDefaultGateway(),
			gevLinkSpeed(),
			gevMessageChannelCount(),
			gevStreamChannelCount(),
			gevSupportedOptionalCommandsUserDefinedName(),
			gevSupportedOptionalCommandsSerialNumber(),
			gevSupportedOptionalCommandsEVENTDATA(),
			gevSupportedOptionalCommandsEVENT(),
			gevSupportedOptionalCommandsPACKETRESEND(),
			gevSupportedOptionalCommandsWRITEMEM(),
			gevSupportedOptionalCommandsConcatenation(),
			gevHeartbeatTimeout(),
			gevTimestampTickFrequency(),
			gevTimestampControlLatch(),
			gevTimestampControlReset(),
			gevTimestampValue(),
			gevDiscoveryAckDelay(),
			gevGVCPExtendedStatusCodes(),
			gevGVCPPendingAck(),
			gevGVCPHeartbeatDisable(),
			gevGVCPPendingTimeout(),
			gevPrimaryApplicationSwitchoverKey(),
			gevCCP(),
			gevPrimaryApplicationSocket(),
			gevPrimaryApplicationIPAddress(),
			gevMCPHostPort(),
			gevMCDA(),
			gevMCTT(),
			gevMCRC(),
			gevMCSP(),
			gevStreamChannelSelector(),
			gevSCCFGUnconditionalStreaming(),
			gevSCCFGExtendedChunkData(),
			gevSCPDirection(),
			gevSCPInterfaceIndex(),
			gevSCPHostPort(),
			gevSCPSFireTestPacket(),
			gevSCPSDoNotFragment(),
			gevSCPSBigEndian(),
			gevSCPSPacketSize(),
			gevSCPD(),
			gevSCDA(),
			gevSCSP(),
			gevManifestEntrySelector(),
			gevManifestXMLMajorVersion(),
			gevManifestXMLMinorVersion(),
			gevManifestXMLSubMinorVersion(),
			gevManifestSchemaMajorVersion(),
			gevManifestSchemaMinorVersion(),
			gevManifestPrimaryURL(),
			gevManifestSecondaryURL(),
			clConfiguration(),
			clTimeSlotsCount(),
			deviceTapGeometry(),
			mvGevSCBWControl(),
			mvGevSCBW()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( payloadSize, "PayloadSize" );
		locator.bindComponent( gevVersionMajor, "GevVersionMajor" );
		locator.bindComponent( gevVersionMinor, "GevVersionMinor" );
		locator.bindComponent( gevDeviceModeIsBigEndian, "GevDeviceModeIsBigEndian" );
		locator.bindComponent( gevDeviceClass, "GevDeviceClass" );
		locator.bindComponent( gevDeviceModeCharacterSet, "GevDeviceModeCharacterSet" );
		locator.bindComponent( gevInterfaceSelector, "GevInterfaceSelector" );
		locator.bindComponent( gevMACAddress, "GevMACAddress" );
		locator.bindComponent( gevSupportedOptionSelector, "GevSupportedOptionSelector" );
		locator.bindComponent( gevSupportedOption, "GevSupportedOption" );
		locator.bindComponent( gevSupportedIPConfigurationLLA, "GevSupportedIPConfigurationLLA" );
		locator.bindComponent( gevSupportedIPConfigurationDHCP, "GevSupportedIPConfigurationDHCP" );
		locator.bindComponent( gevSupportedIPConfigurationPersistentIP, "GevSupportedIPConfigurationPersistentIP" );
		locator.bindComponent( gevCurrentIPConfiguration, "GevCurrentIPConfiguration" );
		locator.bindComponent( gevCurrentIPConfigurationLLA, "GevCurrentIPConfigurationLLA" );
		locator.bindComponent( gevCurrentIPConfigurationDHCP, "GevCurrentIPConfigurationDHCP" );
		locator.bindComponent( gevCurrentIPConfigurationPersistentIP, "GevCurrentIPConfigurationPersistentIP" );
		locator.bindComponent( gevCurrentIPAddress, "GevCurrentIPAddress" );
		locator.bindComponent( gevCurrentSubnetMask, "GevCurrentSubnetMask" );
		locator.bindComponent( gevCurrentDefaultGateway, "GevCurrentDefaultGateway" );
		locator.bindComponent( gevIPConfigurationStatus, "GevIPConfigurationStatus" );
		locator.bindComponent( gevFirstURL, "GevFirstURL" );
		locator.bindComponent( gevSecondURL, "GevSecondURL" );
		locator.bindComponent( gevNumberOfInterfaces, "GevNumberOfInterfaces" );
		locator.bindComponent( gevPersistentIPAddress, "GevPersistentIPAddress" );
		locator.bindComponent( gevPersistentSubnetMask, "GevPersistentSubnetMask" );
		locator.bindComponent( gevPersistentDefaultGateway, "GevPersistentDefaultGateway" );
		locator.bindComponent( gevLinkSpeed, "GevLinkSpeed" );
		locator.bindComponent( gevMessageChannelCount, "GevMessageChannelCount" );
		locator.bindComponent( gevStreamChannelCount, "GevStreamChannelCount" );
		locator.bindComponent( gevSupportedOptionalCommandsUserDefinedName, "GevSupportedOptionalCommandsUserDefinedName" );
		locator.bindComponent( gevSupportedOptionalCommandsSerialNumber, "GevSupportedOptionalCommandsSerialNumber" );
		locator.bindComponent( gevSupportedOptionalCommandsEVENTDATA, "GevSupportedOptionalCommandsEVENTDATA" );
		locator.bindComponent( gevSupportedOptionalCommandsEVENT, "GevSupportedOptionalCommandsEVENT" );
		locator.bindComponent( gevSupportedOptionalCommandsPACKETRESEND, "GevSupportedOptionalCommandsPACKETRESEND" );
		locator.bindComponent( gevSupportedOptionalCommandsWRITEMEM, "GevSupportedOptionalCommandsWRITEMEM" );
		locator.bindComponent( gevSupportedOptionalCommandsConcatenation, "GevSupportedOptionalCommandsConcatenation" );
		locator.bindComponent( gevHeartbeatTimeout, "GevHeartbeatTimeout" );
		locator.bindComponent( gevTimestampTickFrequency, "GevTimestampTickFrequency" );
		locator.bindComponent( gevTimestampControlLatch, "GevTimestampControlLatch@i" );
		locator.bindComponent( gevTimestampControlReset, "GevTimestampControlReset@i" );
		locator.bindComponent( gevTimestampValue, "GevTimestampValue" );
		locator.bindComponent( gevDiscoveryAckDelay, "GevDiscoveryAckDelay" );
		locator.bindComponent( gevGVCPExtendedStatusCodes, "GevGVCPExtendedStatusCodes" );
		locator.bindComponent( gevGVCPPendingAck, "GevGVCPPendingAck" );
		locator.bindComponent( gevGVCPHeartbeatDisable, "GevGVCPHeartbeatDisable" );
		locator.bindComponent( gevGVCPPendingTimeout, "GevGVCPPendingTimeout" );
		locator.bindComponent( gevPrimaryApplicationSwitchoverKey, "GevPrimaryApplicationSwitchoverKey" );
		locator.bindComponent( gevCCP, "GevCCP" );
		locator.bindComponent( gevPrimaryApplicationSocket, "GevPrimaryApplicationSocket" );
		locator.bindComponent( gevPrimaryApplicationIPAddress, "GevPrimaryApplicationIPAddress" );
		locator.bindComponent( gevMCPHostPort, "GevMCPHostPort" );
		locator.bindComponent( gevMCDA, "GevMCDA" );
		locator.bindComponent( gevMCTT, "GevMCTT" );
		locator.bindComponent( gevMCRC, "GevMCRC" );
		locator.bindComponent( gevMCSP, "GevMCSP" );
		locator.bindComponent( gevStreamChannelSelector, "GevStreamChannelSelector" );
		locator.bindComponent( gevSCCFGUnconditionalStreaming, "GevSCCFGUnconditionalStreaming" );
		locator.bindComponent( gevSCCFGExtendedChunkData, "GevSCCFGExtendedChunkData" );
		locator.bindComponent( gevSCPDirection, "GevSCPDirection" );
		locator.bindComponent( gevSCPInterfaceIndex, "GevSCPInterfaceIndex" );
		locator.bindComponent( gevSCPHostPort, "GevSCPHostPort" );
		locator.bindComponent( gevSCPSFireTestPacket, "GevSCPSFireTestPacket" );
		locator.bindComponent( gevSCPSDoNotFragment, "GevSCPSDoNotFragment" );
		locator.bindComponent( gevSCPSBigEndian, "GevSCPSBigEndian" );
		locator.bindComponent( gevSCPSPacketSize, "GevSCPSPacketSize" );
		locator.bindComponent( gevSCPD, "GevSCPD" );
		locator.bindComponent( gevSCDA, "GevSCDA" );
		locator.bindComponent( gevSCSP, "GevSCSP" );
		locator.bindComponent( gevManifestEntrySelector, "GevManifestEntrySelector" );
		locator.bindComponent( gevManifestXMLMajorVersion, "GevManifestXMLMajorVersion" );
		locator.bindComponent( gevManifestXMLMinorVersion, "GevManifestXMLMinorVersion" );
		locator.bindComponent( gevManifestXMLSubMinorVersion, "GevManifestXMLSubMinorVersion" );
		locator.bindComponent( gevManifestSchemaMajorVersion, "GevManifestSchemaMajorVersion" );
		locator.bindComponent( gevManifestSchemaMinorVersion, "GevManifestSchemaMinorVersion" );
		locator.bindComponent( gevManifestPrimaryURL, "GevManifestPrimaryURL" );
		locator.bindComponent( gevManifestSecondaryURL, "GevManifestSecondaryURL" );
		locator.bindComponent( clConfiguration, "ClConfiguration" );
		locator.bindComponent( clTimeSlotsCount, "ClTimeSlotsCount" );
		locator.bindComponent( deviceTapGeometry, "DeviceTapGeometry" );
		locator.bindComponent( mvGevSCBWControl, "mvGevSCBWControl" );
		locator.bindComponent( mvGevSCBW, "mvGevSCBW" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Provides the number of bytes transferred for each image or chunk on the stream channel.
	///
	/// Provides the number of bytes transferred for each image or chunk on the stream channel. This includes any end-of-line, end-of-frame statistics or other stamp data. This is the total size of data payload for a data block.
	PropertyI64 payloadSize;
	/// \brief Major version of the specification.
	///
	/// Major version of the specification.
	PropertyI64 gevVersionMajor;
	/// \brief Minor version of the specification.
	///
	/// Minor version of the specification.
	PropertyI64 gevVersionMinor;
	/// \brief Endianess of the device registers.
	///
	/// Endianess of the device registers.
	PropertyIBoolean gevDeviceModeIsBigEndian;
	/// \brief Returns the class of the device.
	///
	/// Returns the class of the device.
	PropertyI64 gevDeviceClass;
	/// \brief Character set used by all the strings of the bootstrap registers.
	///
	/// Character set used by all the strings of the bootstrap registers.
	PropertyI64 gevDeviceModeCharacterSet;
	/// \brief Selects which physical network interface to control.
	///
	/// Selects which physical network interface to control.
	PropertyI64 gevInterfaceSelector;
	/// \brief MAC address of the network interface.
	///
	/// MAC address of the network interface.
	PropertyI64 gevMACAddress;
	/// \brief Selects the GEV option to interrogate for existing support.
	///
	/// Selects the GEV option to interrogate for existing support.
	PropertyI64 gevSupportedOptionSelector;
	/// \brief Returns if the selected GEV option is supported.
	///
	/// Returns if the selected GEV option is supported.
	PropertyIBoolean gevSupportedOption;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if Link Local Address IP configuration scheme is supported by the given network interface.
	PropertyIBoolean gevSupportedIPConfigurationLLA;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if DHCP IP configuration scheme is supported by the given network interface.
	PropertyIBoolean gevSupportedIPConfigurationDHCP;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if PersistentIP configuration scheme is supported by the given network interface.
	PropertyIBoolean gevSupportedIPConfigurationPersistentIP;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It reports the current IP Configuration scheme. Note that this feature doesn't provision more that one simultaneous IP configuration and should not be used.
	PropertyI64 gevCurrentIPConfiguration;
	/// \brief Controls whether the Link Local Address IP configuration scheme is activated on the given network interface.
	///
	/// Controls whether the Link Local Address IP configuration scheme is activated on the given network interface.
	PropertyIBoolean gevCurrentIPConfigurationLLA;
	/// \brief Controls whether the DHCP IP configuration scheme is activated on the given network interface.
	///
	/// Controls whether the DHCP IP configuration scheme is activated on the given network interface.
	PropertyIBoolean gevCurrentIPConfigurationDHCP;
	/// \brief Controls whether the PersistentIP configuration scheme is activated on the given network interface.
	///
	/// Controls whether the PersistentIP configuration scheme is activated on the given network interface.
	PropertyIBoolean gevCurrentIPConfigurationPersistentIP;
	/// \brief Reports the IP address for the given network interface.
	///
	/// Reports the IP address for the given network interface.
	PropertyI64 gevCurrentIPAddress;
	/// \brief Reports the subnet mask of the given interface.
	///
	/// Reports the subnet mask of the given interface.
	PropertyI64 gevCurrentSubnetMask;
	/// \brief Reports the default gateway IP address to be used on the given network interface.
	///
	/// Reports the default gateway IP address to be used on the given network interface.
	PropertyI64 gevCurrentDefaultGateway;
	/// \brief Reports the current IP configuration status.
	///
	/// Reports the current IP configuration status.
	PropertyI64 gevIPConfigurationStatus;
	/// \brief Indicates the first URL to the XML device description file.
	///
	/// Indicates the first URL to the XML device description file. The First URL is used as the first choice by the application to retrieve the XML device description file.
	PropertyS gevFirstURL;
	/// \brief Indicates the second URL to the XML device description file.
	///
	/// Indicates the second URL to the XML device description file. This URL is an alternative if the application was unsuccessful to retrieve the device description file using the first URL.
	PropertyS gevSecondURL;
	/// \brief Indicates the number of physical network interfaces supported by this device.
	///
	/// Indicates the number of physical network interfaces supported by this device.
	PropertyI64 gevNumberOfInterfaces;
	/// \brief Controls the Persistent IP address for this network interface.
	///
	/// Controls the Persistent IP address for this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
	PropertyI64 gevPersistentIPAddress;
	/// \brief Controls the Persistent subnet mask associated with the Persistent IP address on this network interface.
	///
	/// Controls the Persistent subnet mask associated with the Persistent IP address on this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
	PropertyI64 gevPersistentSubnetMask;
	/// \brief Controls the persistent default gateway for this network interface.
	///
	/// Controls the persistent default gateway for this network interface. It is only used when the device boots with the Persistent IP configuration scheme.
	PropertyI64 gevPersistentDefaultGateway;
	/// \brief Indicates the speed of transmission negotiated by the given network interface.
	///
	/// Indicates the speed of transmission negotiated by the given network interface.
	PropertyI64 gevLinkSpeed;
	/// \brief Indicates the number of message channels supported by this device.
	///
	/// Indicates the number of message channels supported by this device.
	PropertyI64 gevMessageChannelCount;
	/// \brief Indicates the number of stream channels supported by this device.
	///
	/// Indicates the number of stream channels supported by this device.
	PropertyI64 gevStreamChannelCount;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the User-defined name register is supported.
	PropertyIBoolean gevSupportedOptionalCommandsUserDefinedName;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the Serial number register is supported.
	PropertyIBoolean gevSupportedOptionalCommandsSerialNumber;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the EVENTDATA_CMD and EVENTDATA_ACK are supported.
	PropertyIBoolean gevSupportedOptionalCommandsEVENTDATA;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the EVENT_CMD and EVENT_ACK are supported.
	PropertyIBoolean gevSupportedOptionalCommandsEVENT;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the PACKETRESEND_CMD is supported.
	PropertyIBoolean gevSupportedOptionalCommandsPACKETRESEND;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the WRITEMEM_CMD and WRITEMEM_ACK are supported.
	PropertyIBoolean gevSupportedOptionalCommandsWRITEMEM;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. GevSupportedOption should be used instead. It indicates if the Multiple operations in a single message are supported.
	PropertyIBoolean gevSupportedOptionalCommandsConcatenation;
	/// \brief Controls the current heartbeat timeout in milliseconds.
	///
	/// Controls the current heartbeat timeout in milliseconds.
	PropertyI64 gevHeartbeatTimeout;
	/// \brief Indicates the number of timestamp ticks in 1 second (frequency in Hz).
	///
	/// Indicates the number of timestamp ticks in 1 second (frequency in Hz).
	PropertyI64 gevTimestampTickFrequency;
	/// \brief Latches the current timestamp counter into GevTimestampValue.
	///
	/// Latches the current timestamp counter into GevTimestampValue.
	Method gevTimestampControlLatch;
	/// \brief Resets the timestamp counter to 0.
	///
	/// Resets the timestamp counter to 0.
	Method gevTimestampControlReset;
	/// \brief Returns the latched 64-bit value of the timestamp counter.
	///
	/// Returns the latched 64-bit value of the timestamp counter.
	PropertyI64 gevTimestampValue;
	/// \brief Indicates the maximum randomized delay the device will wait to acknowledge a discovery command.
	///
	/// Indicates the maximum randomized delay the device will wait to acknowledge a discovery command.
	PropertyI64 gevDiscoveryAckDelay;
	/// \brief Enables the generation of extended status codes.
	///
	/// Enables the generation of extended status codes.
	PropertyIBoolean gevGVCPExtendedStatusCodes;
	/// \brief Enables the generation of PENDING_ACK.
	///
	/// Enables the generation of PENDING_ACK.
	PropertyIBoolean gevGVCPPendingAck;
	/// \brief Disables the GVCP heartbeat.
	///
	/// Disables the GVCP heartbeat.
	PropertyIBoolean gevGVCPHeartbeatDisable;
	/// \brief Indicates the longest GVCP command execution time before a device returns a PENDING_ACK.
	///
	/// Indicates the longest GVCP command execution time before a device returns a PENDING_ACK.
	PropertyI64 gevGVCPPendingTimeout;
	/// \brief Controls the key to use to authenticate primary application switchover requests.
	///
	/// Controls the key to use to authenticate primary application switchover requests.
	PropertyI64 gevPrimaryApplicationSwitchoverKey;
	/// \brief Controls the device access privilege of an application.
	///
	/// Controls the device access privilege of an application.
	PropertyI64 gevCCP;
	/// \brief Returns the UDP source port of the primary application.
	///
	/// Returns the UDP source port of the primary application.
	PropertyI64 gevPrimaryApplicationSocket;
	/// \brief Returns the address of the primary application.
	///
	/// Returns the address of the primary application.
	PropertyI64 gevPrimaryApplicationIPAddress;
	/// \brief Controls the port to which the device must send messages.
	///
	/// Controls the port to which the device must send messages. Setting this value to 0 closes the message channel.
	PropertyI64 gevMCPHostPort;
	/// \brief Controls the destination IP address for the message channel.
	///
	/// Controls the destination IP address for the message channel.
	PropertyI64 gevMCDA;
	/// \brief Provides the transmission timeout value in milliseconds.
	///
	/// Provides the transmission timeout value in milliseconds.
	PropertyI64 gevMCTT;
	/// \brief Controls the number of retransmissions allowed when a message channel message times out.
	///
	/// Controls the number of retransmissions allowed when a message channel message times out.
	PropertyI64 gevMCRC;
	/// \brief This feature indicates the source port for the message channel.
	///
	/// This feature indicates the source port for the message channel.
	PropertyI64 gevMCSP;
	/// \brief Selects the stream channel to control.
	///
	/// Selects the stream channel to control.
	PropertyI64 gevStreamChannelSelector;
	/// \brief Enables the camera to continue to stream, for this stream channel, if its control channel is closed or regardless of the reception of any ICMP messages (such as destination unreachable messages).
	///
	/// Enables the camera to continue to stream, for this stream channel, if its control channel is closed or regardless of the reception of any ICMP messages (such as destination unreachable messages).
	PropertyIBoolean gevSCCFGUnconditionalStreaming;
	/// \brief Enables cameras to use the extended chunk data payload type for this stream channel.
	///
	/// Enables cameras to use the extended chunk data payload type for this stream channel.
	PropertyIBoolean gevSCCFGExtendedChunkData;
	/// \brief Reports the direction of the stream channel.
	///
	/// Reports the direction of the stream channel.
	PropertyI64 gevSCPDirection;
	/// \brief Index of network interface to use.
	///
	/// Index of network interface to use.
	PropertyI64 gevSCPInterfaceIndex;
	/// \brief Controls the port of the selected channel to which a GVSP transmitter must send data stream or the port from which a GVSP receiver may receive data stream.
	///
	/// Controls the port of the selected channel to which a GVSP transmitter must send data stream or the port from which a GVSP receiver may receive data stream. Setting this value to 0 closes the stream channel.
	PropertyI64 gevSCPHostPort;
	/// \brief Sends a test packet.
	///
	/// Sends a test packet. When this feature is set, the device will fire one test packet.
	PropertyIBoolean gevSCPSFireTestPacket;
	/// \brief The state of this feature is copied into the 'do not fragment' bit of IP header of each stream packet.
	///
	/// The state of this feature is copied into the 'do not fragment' bit of IP header of each stream packet. It can be used by the application to prevent IP fragmentation of packets on the stream channel.
	PropertyIBoolean gevSCPSDoNotFragment;
	/// \brief Endianess of multi-byte pixel data for this stream.
	///
	/// Endianess of multi-byte pixel data for this stream.
	PropertyIBoolean gevSCPSBigEndian;
	/// \brief Specifies the stream packet size, in bytes, to send on the selected channel for a GVSP transmitter or specifies the maximum packet size supported by a GVSP receiver.
	///
	/// Specifies the stream packet size, in bytes, to send on the selected channel for a GVSP transmitter or specifies the maximum packet size supported by a GVSP receiver.
	PropertyI64 gevSCPSPacketSize;
	/// \brief Controls the delay (in timestamp counter unit) to insert between each packet for this stream channel.
	///
	/// Controls the delay (in timestamp counter unit) to insert between each packet for this stream channel. This can be used as a crude flow-control mechanism if the application or the network infrastructure cannot keep up with the packets coming from the device.
	PropertyI64 gevSCPD;
	/// \brief Controls the destination IP address of the selected stream channel to which a GVSP transmitter must send data stream or the destination IP address from which a GVSP receiver may receive data stream.
	///
	/// Controls the destination IP address of the selected stream channel to which a GVSP transmitter must send data stream or the destination IP address from which a GVSP receiver may receive data stream.
	PropertyI64 gevSCDA;
	/// \brief Indicates the source port of the stream channel.
	///
	/// Indicates the source port of the stream channel.
	PropertyI64 gevSCSP;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Selects the manifest entry to reference.
	PropertyI64 gevManifestEntrySelector;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the major version number of the XML file of the selected manifest entry.
	PropertyI64 gevManifestXMLMajorVersion;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the minor version number of the XML file of the selected manifest entry.
	PropertyI64 gevManifestXMLMinorVersion;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the subminor version number of the XML file of the selected manifest entry.
	PropertyI64 gevManifestXMLSubMinorVersion;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the major version number of the schema file of the selected manifest entry.
	PropertyI64 gevManifestSchemaMajorVersion;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent.. Indicates the minor version number of the schema file of the selected manifest entry.
	PropertyI64 gevManifestSchemaMinorVersion;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the first URL to the XML device description file of the selected manifest entry.
	PropertyS gevManifestPrimaryURL;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. See the Device Control section for an equivalent. Indicates the second URL to the XML device description file of the selected manifest entry.
	PropertyS gevManifestSecondaryURL;
	/// \brief This Camera Link specific feature describes the configuration used by the camera.
	///
	/// This Camera Link specific feature describes the configuration used by the camera. It helps especially when a camera is capable of operation in a non-standard configuration, and when the features PixelSize, SensorDigitizationTaps, and DeviceTapGeometry do not provide enough information for interpretation of the image data provided by the camera.
	PropertyI64 clConfiguration;
	/// \brief This Camera Link specific feature describes the time multiplexing of the camera link connection to transfer more than the configuration allows, in one single clock.
	///
	/// This Camera Link specific feature describes the time multiplexing of the camera link connection to transfer more than the configuration allows, in one single clock.
	PropertyI64 clTimeSlotsCount;
	/// \brief This device tap geometry feature describes the geometrical properties characterizing the taps of a camera as seen from the frame grabber or acquisition card.
	///
	/// This device tap geometry feature describes the geometrical properties characterizing the taps of a camera as seen from the frame grabber or acquisition card. Note the case of RGB where even though there are 3 color components, they are considered to be one tap. This feature is mainly applicable to Camera link cameras.
	PropertyI64 deviceTapGeometry;
	/// \brief Selects the bandwidth control for the selected stream channel.
	///
	/// This enumeration selects the bandwidth control for the selected stream channel.
	PropertyI64 mvGevSCBWControl;
	/// \brief Sets the stream channels max. bandwidth in KBps
	///
	/// This value sets the stream channels max. bandwidth in KBps.
	PropertyI64 mvGevSCBW;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getPayloadSize( void ) const { return payloadSize; }
	PropertyI64 getGevVersionMajor( void ) const { return gevVersionMajor; }
	PropertyI64 getGevVersionMinor( void ) const { return gevVersionMinor; }
	PropertyIBoolean getGevDeviceModeIsBigEndian( void ) const { return gevDeviceModeIsBigEndian; }
	PropertyI64 getGevDeviceClass( void ) const { return gevDeviceClass; }
	PropertyI64 getGevDeviceModeCharacterSet( void ) const { return gevDeviceModeCharacterSet; }
	PropertyI64 getGevInterfaceSelector( void ) const { return gevInterfaceSelector; }
	PropertyI64 getGevMACAddress( void ) const { return gevMACAddress; }
	PropertyI64 getGevSupportedOptionSelector( void ) const { return gevSupportedOptionSelector; }
	PropertyIBoolean getGevSupportedOption( void ) const { return gevSupportedOption; }
	PropertyIBoolean getGevSupportedIPConfigurationLLA( void ) const { return gevSupportedIPConfigurationLLA; }
	PropertyIBoolean getGevSupportedIPConfigurationDHCP( void ) const { return gevSupportedIPConfigurationDHCP; }
	PropertyIBoolean getGevSupportedIPConfigurationPersistentIP( void ) const { return gevSupportedIPConfigurationPersistentIP; }
	PropertyI64 getGevCurrentIPConfiguration( void ) const { return gevCurrentIPConfiguration; }
	PropertyIBoolean getGevCurrentIPConfigurationLLA( void ) const { return gevCurrentIPConfigurationLLA; }
	PropertyIBoolean getGevCurrentIPConfigurationDHCP( void ) const { return gevCurrentIPConfigurationDHCP; }
	PropertyIBoolean getGevCurrentIPConfigurationPersistentIP( void ) const { return gevCurrentIPConfigurationPersistentIP; }
	PropertyI64 getGevCurrentIPAddress( void ) const { return gevCurrentIPAddress; }
	PropertyI64 getGevCurrentSubnetMask( void ) const { return gevCurrentSubnetMask; }
	PropertyI64 getGevCurrentDefaultGateway( void ) const { return gevCurrentDefaultGateway; }
	PropertyI64 getGevIPConfigurationStatus( void ) const { return gevIPConfigurationStatus; }
	PropertyS getGevFirstURL( void ) const { return gevFirstURL; }
	PropertyS getGevSecondURL( void ) const { return gevSecondURL; }
	PropertyI64 getGevNumberOfInterfaces( void ) const { return gevNumberOfInterfaces; }
	PropertyI64 getGevPersistentIPAddress( void ) const { return gevPersistentIPAddress; }
	PropertyI64 getGevPersistentSubnetMask( void ) const { return gevPersistentSubnetMask; }
	PropertyI64 getGevPersistentDefaultGateway( void ) const { return gevPersistentDefaultGateway; }
	PropertyI64 getGevLinkSpeed( void ) const { return gevLinkSpeed; }
	PropertyI64 getGevMessageChannelCount( void ) const { return gevMessageChannelCount; }
	PropertyI64 getGevStreamChannelCount( void ) const { return gevStreamChannelCount; }
	PropertyIBoolean getGevSupportedOptionalCommandsUserDefinedName( void ) const { return gevSupportedOptionalCommandsUserDefinedName; }
	PropertyIBoolean getGevSupportedOptionalCommandsSerialNumber( void ) const { return gevSupportedOptionalCommandsSerialNumber; }
	PropertyIBoolean getGevSupportedOptionalCommandsEVENTDATA( void ) const { return gevSupportedOptionalCommandsEVENTDATA; }
	PropertyIBoolean getGevSupportedOptionalCommandsEVENT( void ) const { return gevSupportedOptionalCommandsEVENT; }
	PropertyIBoolean getGevSupportedOptionalCommandsPACKETRESEND( void ) const { return gevSupportedOptionalCommandsPACKETRESEND; }
	PropertyIBoolean getGevSupportedOptionalCommandsWRITEMEM( void ) const { return gevSupportedOptionalCommandsWRITEMEM; }
	PropertyIBoolean getGevSupportedOptionalCommandsConcatenation( void ) const { return gevSupportedOptionalCommandsConcatenation; }
	PropertyI64 getGevHeartbeatTimeout( void ) const { return gevHeartbeatTimeout; }
	PropertyI64 getGevTimestampTickFrequency( void ) const { return gevTimestampTickFrequency; }
	Method getGevTimestampControlLatch( void ) const { return gevTimestampControlLatch; }
	Method getGevTimestampControlReset( void ) const { return gevTimestampControlReset; }
	PropertyI64 getGevTimestampValue( void ) const { return gevTimestampValue; }
	PropertyI64 getGevDiscoveryAckDelay( void ) const { return gevDiscoveryAckDelay; }
	PropertyIBoolean getGevGVCPExtendedStatusCodes( void ) const { return gevGVCPExtendedStatusCodes; }
	PropertyIBoolean getGevGVCPPendingAck( void ) const { return gevGVCPPendingAck; }
	PropertyIBoolean getGevGVCPHeartbeatDisable( void ) const { return gevGVCPHeartbeatDisable; }
	PropertyI64 getGevGVCPPendingTimeout( void ) const { return gevGVCPPendingTimeout; }
	PropertyI64 getGevPrimaryApplicationSwitchoverKey( void ) const { return gevPrimaryApplicationSwitchoverKey; }
	PropertyI64 getGevCCP( void ) const { return gevCCP; }
	PropertyI64 getGevPrimaryApplicationSocket( void ) const { return gevPrimaryApplicationSocket; }
	PropertyI64 getGevPrimaryApplicationIPAddress( void ) const { return gevPrimaryApplicationIPAddress; }
	PropertyI64 getGevMCPHostPort( void ) const { return gevMCPHostPort; }
	PropertyI64 getGevMCDA( void ) const { return gevMCDA; }
	PropertyI64 getGevMCTT( void ) const { return gevMCTT; }
	PropertyI64 getGevMCRC( void ) const { return gevMCRC; }
	PropertyI64 getGevMCSP( void ) const { return gevMCSP; }
	PropertyI64 getGevStreamChannelSelector( void ) const { return gevStreamChannelSelector; }
	PropertyIBoolean getGevSCCFGUnconditionalStreaming( void ) const { return gevSCCFGUnconditionalStreaming; }
	PropertyIBoolean getGevSCCFGExtendedChunkData( void ) const { return gevSCCFGExtendedChunkData; }
	PropertyI64 getGevSCPDirection( void ) const { return gevSCPDirection; }
	PropertyI64 getGevSCPInterfaceIndex( void ) const { return gevSCPInterfaceIndex; }
	PropertyI64 getGevSCPHostPort( void ) const { return gevSCPHostPort; }
	PropertyIBoolean getGevSCPSFireTestPacket( void ) const { return gevSCPSFireTestPacket; }
	PropertyIBoolean getGevSCPSDoNotFragment( void ) const { return gevSCPSDoNotFragment; }
	PropertyIBoolean getGevSCPSBigEndian( void ) const { return gevSCPSBigEndian; }
	PropertyI64 getGevSCPSPacketSize( void ) const { return gevSCPSPacketSize; }
	PropertyI64 getGevSCPD( void ) const { return gevSCPD; }
	PropertyI64 getGevSCDA( void ) const { return gevSCDA; }
	PropertyI64 getGevSCSP( void ) const { return gevSCSP; }
	PropertyI64 getGevManifestEntrySelector( void ) const { return gevManifestEntrySelector; }
	PropertyI64 getGevManifestXMLMajorVersion( void ) const { return gevManifestXMLMajorVersion; }
	PropertyI64 getGevManifestXMLMinorVersion( void ) const { return gevManifestXMLMinorVersion; }
	PropertyI64 getGevManifestXMLSubMinorVersion( void ) const { return gevManifestXMLSubMinorVersion; }
	PropertyI64 getGevManifestSchemaMajorVersion( void ) const { return gevManifestSchemaMajorVersion; }
	PropertyI64 getGevManifestSchemaMinorVersion( void ) const { return gevManifestSchemaMinorVersion; }
	PropertyS getGevManifestPrimaryURL( void ) const { return gevManifestPrimaryURL; }
	PropertyS getGevManifestSecondaryURL( void ) const { return gevManifestSecondaryURL; }
	PropertyI64 getClConfiguration( void ) const { return clConfiguration; }
	PropertyI64 getClTimeSlotsCount( void ) const { return clTimeSlotsCount; }
	PropertyI64 getDeviceTapGeometry( void ) const { return deviceTapGeometry; }
	PropertyI64 getmvGevSCBWControl( void ) const { return mvGevSCBWControl; }
	PropertyI64 getmvGevSCBW( void ) const { return mvGevSCBW; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the User Set control features.
///
/// Category that contains the User Set control features.
class UserSetControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::UserSetControl</b> object.
	explicit UserSetControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			userSetSelector(),
			userSetLoad(),
			userSetSave(),
			userSetDefaultSelector(),
			mvUserData()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( userSetSelector, "UserSetSelector" );
		locator.bindComponent( userSetLoad, "UserSetLoad@i" );
		locator.bindComponent( userSetSave, "UserSetSave@i" );
		locator.bindComponent( userSetDefaultSelector, "UserSetDefaultSelector" );
		locator.bindComponent( mvUserData, "mvUserData" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects the feature User Set to load, save or configure.
	///
	/// Selects the feature User Set to load, save or configure.
	PropertyI64 userSetSelector;
	/// \brief Loads the User Set specified by UserSetSelector to the device and makes it active.
	///
	/// Loads the User Set specified by UserSetSelector to the device and makes it active.
	Method userSetLoad;
	/// \brief Save the User Set specified by UserSetSelector to the non-volatile memory of the device.
	///
	/// Save the User Set specified by UserSetSelector to the non-volatile memory of the device.
	Method userSetSave;
	/// \brief Selects the feature User Set to load and make active when the device is reset.
	///
	/// Selects the feature User Set to load and make active when the device is reset.
	PropertyI64 userSetDefaultSelector;
	/// \brief A register to store arbitrary user data into the devices non-volatile memory.
	///
	/// A register to store arbitrary user data into the devices non-volatile memory
	PropertyS mvUserData;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getUserSetSelector( void ) const { return userSetSelector; }
	Method getUserSetLoad( void ) const { return userSetLoad; }
	Method getUserSetSave( void ) const { return userSetSave; }
	PropertyI64 getUserSetDefaultSelector( void ) const { return userSetDefaultSelector; }
	PropertyS getmvUserData( void ) const { return mvUserData; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the Chunk Data control features.
///
/// Category that contains the Chunk Data control features.
class ChunkDataControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::ChunkDataControl</b> object.
	explicit ChunkDataControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			chunkModeActive(),
			chunkSelector(),
			chunkEnable(),
			chunkImage(),
			chunkOffsetX(),
			chunkOffsetY(),
			chunkWidth(),
			chunkHeight(),
			chunkPixelFormat(),
			chunkPixelDynamicRangeMin(),
			chunkPixelDynamicRangeMax(),
			chunkDynamicRangeMin(),
			chunkDynamicRangeMax(),
			chunkTimestamp(),
			chunkLineStatusAll(),
			chunkCounterSelector(),
			chunkCounterValue(),
			chunkCounter(),
			chunkTimerSelector(),
			chunkTimerValue(),
			chunkTimer(),
			chunkExposureTime(),
			chunkGainSelector(),
			chunkGain(),
			chunkBlackLevelSelector(),
			chunkBlackLevel(),
			chunkLinePitch(),
			chunkFrameID()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( chunkModeActive, "ChunkModeActive" );
		locator.bindComponent( chunkSelector, "ChunkSelector" );
		locator.bindComponent( chunkEnable, "ChunkEnable" );
		locator.bindComponent( chunkImage, "ChunkImage" );
		locator.bindComponent( chunkOffsetX, "ChunkOffsetX" );
		locator.bindComponent( chunkOffsetY, "ChunkOffsetY" );
		locator.bindComponent( chunkWidth, "ChunkWidth" );
		locator.bindComponent( chunkHeight, "ChunkHeight" );
		locator.bindComponent( chunkPixelFormat, "ChunkPixelFormat" );
		locator.bindComponent( chunkPixelDynamicRangeMin, "ChunkPixelDynamicRangeMin" );
		locator.bindComponent( chunkPixelDynamicRangeMax, "ChunkPixelDynamicRangeMax" );
		locator.bindComponent( chunkDynamicRangeMin, "ChunkDynamicRangeMin" );
		locator.bindComponent( chunkDynamicRangeMax, "ChunkDynamicRangeMax" );
		locator.bindComponent( chunkTimestamp, "ChunkTimestamp" );
		locator.bindComponent( chunkLineStatusAll, "ChunkLineStatusAll" );
		locator.bindComponent( chunkCounterSelector, "ChunkCounterSelector" );
		locator.bindComponent( chunkCounterValue, "ChunkCounterValue" );
		locator.bindComponent( chunkCounter, "ChunkCounter" );
		locator.bindComponent( chunkTimerSelector, "ChunkTimerSelector" );
		locator.bindComponent( chunkTimerValue, "ChunkTimerValue" );
		locator.bindComponent( chunkTimer, "ChunkTimer" );
		locator.bindComponent( chunkExposureTime, "ChunkExposureTime" );
		locator.bindComponent( chunkGainSelector, "ChunkGainSelector" );
		locator.bindComponent( chunkGain, "ChunkGain" );
		locator.bindComponent( chunkBlackLevelSelector, "ChunkBlackLevelSelector" );
		locator.bindComponent( chunkBlackLevel, "ChunkBlackLevel" );
		locator.bindComponent( chunkLinePitch, "ChunkLinePitch" );
		locator.bindComponent( chunkFrameID, "ChunkFrameID" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Activates the inclusion of Chunk data in the payload of the image.
	///
	/// Activates the inclusion of Chunk data in the payload of the image.
	PropertyIBoolean chunkModeActive;
	/// \brief Selects which Chunk to enable or control.
	///
	/// Selects which Chunk to enable or control.
	PropertyI64 chunkSelector;
	/// \brief Enables the inclusion of the selected Chunk data in the payload of the image.
	///
	/// Enables the inclusion of the selected Chunk data in the payload of the image.
	PropertyIBoolean chunkEnable;
	/// \brief Returns the entire image data included in the payload.
	///
	/// Returns the entire image data included in the payload.
	PropertyS chunkImage;
	/// \brief Returns the OffsetX of the image included in the payload.
	///
	/// Returns the OffsetX of the image included in the payload.
	PropertyI64 chunkOffsetX;
	/// \brief Returns the OffsetY of the image included in the payload.
	///
	/// Returns the OffsetY of the image included in the payload.
	PropertyI64 chunkOffsetY;
	/// \brief Returns the Width of the image included in the payload.
	///
	/// Returns the Width of the image included in the payload.
	PropertyI64 chunkWidth;
	/// \brief Returns the Height of the image included in the payload.
	///
	/// Returns the Height of the image included in the payload.
	PropertyI64 chunkHeight;
	/// \brief Returns the PixelFormat of the image included in the payload.
	///
	/// Returns the PixelFormat of the image included in the payload.
	PropertyI64 chunkPixelFormat;
	/// \brief Returns the minimum value of dynamic range of the image included in the payload.
	///
	/// Returns the minimum value of dynamic range of the image included in the payload.
	PropertyI64 chunkPixelDynamicRangeMin;
	/// \brief Returns the maximum value of dynamic range of the image included in the payload.
	///
	/// Returns the maximum value of dynamic range of the image included in the payload.
	PropertyI64 chunkPixelDynamicRangeMax;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. ChunkPixelDynamicRangeMin should be used instead. Returns the minimum value of dynamic range of the image included in the payload.
	PropertyI64 chunkDynamicRangeMin;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. ChunkPixelDynamicRangeMax should be used instead. Returns the maximum value of dynamic range of the image included in the payload.
	PropertyI64 chunkDynamicRangeMax;
	/// \brief Returns the Timestamp of the image included in the payload at the time of the FrameStart internal event.
	///
	/// Returns the Timestamp of the image included in the payload at the time of the FrameStart internal event.
	PropertyI64 chunkTimestamp;
	/// \brief Returns the status of all the I/O lines at the time of the FrameStart internal event.
	///
	/// Returns the status of all the I/O lines at the time of the FrameStart internal event.
	PropertyI64 chunkLineStatusAll;
	/// \brief Selects which counter to retrieve data from.
	///
	/// Selects which counter to retrieve data from.
	PropertyI64 chunkCounterSelector;
	/// \brief Returns the value of the selected Chunk counter at the time of the FrameStart event.
	///
	/// Returns the value of the selected Chunk counter at the time of the FrameStart event.
	PropertyI64 chunkCounterValue;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. Returns the value of the selected Chunk counter at the time of the FrameStart internal event.
	PropertyI64 chunkCounter;
	/// \brief Selects which Timer to retrieve data from.
	///
	/// Selects which Timer to retrieve data from.
	PropertyI64 chunkTimerSelector;
	/// \brief Returns the value of the selected Timer at the time of the FrameStart internal event.
	///
	/// Returns the value of the selected Timer at the time of the FrameStart internal event.
	PropertyF chunkTimerValue;
	/// \brief This feature is deprecated.
	///
	/// This feature is deprecated. Returns the value of the selected Timer at the time of the FrameStart internal event.
	PropertyF chunkTimer;
	/// \brief Returns the exposure time used to capture the image.
	///
	/// Returns the exposure time used to capture the image.
	PropertyF chunkExposureTime;
	/// \brief Selects which Gain to retrieve data from.
	///
	/// Selects which Gain to retrieve data from.
	PropertyI64 chunkGainSelector;
	/// \brief Returns the gain used to capture the image.
	///
	/// Returns the gain used to capture the image.
	PropertyF chunkGain;
	/// \brief Selects which Black Level to retrieve data from.
	///
	/// Selects which Black Level to retrieve data from.
	PropertyI64 chunkBlackLevelSelector;
	/// \brief Returns the black level used to capture the image included in the payload.
	///
	/// Returns the black level used to capture the image included in the payload.
	PropertyF chunkBlackLevel;
	/// \brief Returns the LinePitch of the image included in the payload.
	///
	/// Returns the LinePitch of the image included in the payload.
	PropertyI64 chunkLinePitch;
	/// \brief Returns the unique Identifier of the frame (or image) included in the payload.
	///
	/// Returns the unique Identifier of the frame (or image) included in the payload.
	PropertyI64 chunkFrameID;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyIBoolean getChunkModeActive( void ) const { return chunkModeActive; }
	PropertyI64 getChunkSelector( void ) const { return chunkSelector; }
	PropertyIBoolean getChunkEnable( void ) const { return chunkEnable; }
	PropertyS getChunkImage( void ) const { return chunkImage; }
	PropertyI64 getChunkOffsetX( void ) const { return chunkOffsetX; }
	PropertyI64 getChunkOffsetY( void ) const { return chunkOffsetY; }
	PropertyI64 getChunkWidth( void ) const { return chunkWidth; }
	PropertyI64 getChunkHeight( void ) const { return chunkHeight; }
	PropertyI64 getChunkPixelFormat( void ) const { return chunkPixelFormat; }
	PropertyI64 getChunkPixelDynamicRangeMin( void ) const { return chunkPixelDynamicRangeMin; }
	PropertyI64 getChunkPixelDynamicRangeMax( void ) const { return chunkPixelDynamicRangeMax; }
	PropertyI64 getChunkDynamicRangeMin( void ) const { return chunkDynamicRangeMin; }
	PropertyI64 getChunkDynamicRangeMax( void ) const { return chunkDynamicRangeMax; }
	PropertyI64 getChunkTimestamp( void ) const { return chunkTimestamp; }
	PropertyI64 getChunkLineStatusAll( void ) const { return chunkLineStatusAll; }
	PropertyI64 getChunkCounterSelector( void ) const { return chunkCounterSelector; }
	PropertyI64 getChunkCounterValue( void ) const { return chunkCounterValue; }
	PropertyI64 getChunkCounter( void ) const { return chunkCounter; }
	PropertyI64 getChunkTimerSelector( void ) const { return chunkTimerSelector; }
	PropertyF getChunkTimerValue( void ) const { return chunkTimerValue; }
	PropertyF getChunkTimer( void ) const { return chunkTimer; }
	PropertyF getChunkExposureTime( void ) const { return chunkExposureTime; }
	PropertyI64 getChunkGainSelector( void ) const { return chunkGainSelector; }
	PropertyF getChunkGain( void ) const { return chunkGain; }
	PropertyI64 getChunkBlackLevelSelector( void ) const { return chunkBlackLevelSelector; }
	PropertyF getChunkBlackLevel( void ) const { return chunkBlackLevel; }
	PropertyI64 getChunkLinePitch( void ) const { return chunkLinePitch; }
	PropertyI64 getChunkFrameID( void ) const { return chunkFrameID; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the File Access control features.
///
/// Category that contains the File Access control features.
class FileAccessControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::FileAccessControl</b> object.
	explicit FileAccessControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			fileSelector(),
			fileOperationSelector(),
			fileOperationExecute(),
			fileOpenMode(),
			fileAccessBuffer(),
			fileAccessOffset(),
			fileAccessLength(),
			fileOperationStatus(),
			fileOperationResult(),
			fileSize()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( fileSelector, "FileSelector" );
		locator.bindComponent( fileOperationSelector, "FileOperationSelector" );
		locator.bindComponent( fileOperationExecute, "FileOperationExecute@i" );
		locator.bindComponent( fileOpenMode, "FileOpenMode" );
		locator.bindComponent( fileAccessBuffer, "FileAccessBuffer" );
		locator.bindComponent( fileAccessOffset, "FileAccessOffset" );
		locator.bindComponent( fileAccessLength, "FileAccessLength" );
		locator.bindComponent( fileOperationStatus, "FileOperationStatus" );
		locator.bindComponent( fileOperationResult, "FileOperationResult" );
		locator.bindComponent( fileSize, "FileSize" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects the target file in the device.
	///
	/// Selects the target file in the device.
	PropertyI64 fileSelector;
	/// \brief Selects the target operation for the selected file in the device.
	///
	/// Selects the target operation for the selected file in the device. This Operation is executed when the FileOperationExecute feature is called.
	PropertyI64 fileOperationSelector;
	/// \brief Executes the operation selected by FileOperationSelector on the selected file.
	///
	/// Executes the operation selected by FileOperationSelector on the selected file.
	Method fileOperationExecute;
	/// \brief Selects the access mode in which a file is opened in the device.
	///
	/// Selects the access mode in which a file is opened in the device.
	PropertyI64 fileOpenMode;
	/// \brief Defines the intermediate access buffer that allows the exchange of data between the device file storage and the application.
	///
	/// Defines the intermediate access buffer that allows the exchange of data between the device file storage and the application.
	PropertyS fileAccessBuffer;
	/// \brief Controls the Offset of the mapping between the device file storage and the FileAccessBuffer.
	///
	/// Controls the Offset of the mapping between the device file storage and the FileAccessBuffer.
	PropertyI64 fileAccessOffset;
	/// \brief Controls the Length of the mapping between the device file storage and the FileAccessBuffer.
	///
	/// Controls the Length of the mapping between the device file storage and the FileAccessBuffer.
	PropertyI64 fileAccessLength;
	/// \brief Represents the file operation execution status.
	///
	/// Represents the file operation execution status.
	PropertyI64 fileOperationStatus;
	/// \brief Represents the file operation result.
	///
	/// Represents the file operation result. For Read or Write operations, the number of successfully read/written bytes is returned.
	PropertyI64 fileOperationResult;
	/// \brief Represents the size of the selected file in bytes.
	///
	/// Represents the size of the selected file in bytes.
	PropertyI64 fileSize;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getFileSelector( void ) const { return fileSelector; }
	PropertyI64 getFileOperationSelector( void ) const { return fileOperationSelector; }
	Method getFileOperationExecute( void ) const { return fileOperationExecute; }
	PropertyI64 getFileOpenMode( void ) const { return fileOpenMode; }
	PropertyS getFileAccessBuffer( void ) const { return fileAccessBuffer; }
	PropertyI64 getFileAccessOffset( void ) const { return fileAccessOffset; }
	PropertyI64 getFileAccessLength( void ) const { return fileAccessLength; }
	PropertyI64 getFileOperationStatus( void ) const { return fileOperationStatus; }
	PropertyI64 getFileOperationResult( void ) const { return fileOperationResult; }
	PropertyI64 getFileSize( void ) const { return fileSize; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the Color Transformation control features.
///
/// Category that contains the Color Transformation control features.
class ColorTransformationControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::ColorTransformationControl</b> object.
	explicit ColorTransformationControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			colorTransformationSelector(),
			colorTransformationEnable(),
			colorTransformationValueSelector(),
			colorTransformationValue()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( colorTransformationSelector, "ColorTransformationSelector" );
		locator.bindComponent( colorTransformationEnable, "ColorTransformationEnable" );
		locator.bindComponent( colorTransformationValueSelector, "ColorTransformationValueSelector" );
		locator.bindComponent( colorTransformationValue, "ColorTransformationValue" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects which Color Transformation module is controlled by the various Color Transformation features.
	///
	/// Selects which Color Transformation module is controlled by the various Color Transformation features.
	PropertyI64 colorTransformationSelector;
	/// \brief Activates the selected Color Transformation module.
	///
	/// Activates the selected Color Transformation module.
	PropertyIBoolean colorTransformationEnable;
	/// \brief Selects the Gain factor or Offset of the Transformation matrix to access in the selected Color Transformation module.
	///
	/// Selects the Gain factor or Offset of the Transformation matrix to access in the selected Color Transformation module.
	PropertyI64 colorTransformationValueSelector;
	/// \brief Represents the value of the selected Gain factor or Offset inside the Transformation matrix.
	///
	/// Represents the value of the selected Gain factor or Offset inside the Transformation matrix.
	PropertyF colorTransformationValue;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getColorTransformationSelector( void ) const { return colorTransformationSelector; }
	PropertyIBoolean getColorTransformationEnable( void ) const { return colorTransformationEnable; }
	PropertyI64 getColorTransformationValueSelector( void ) const { return colorTransformationValueSelector; }
	PropertyF getColorTransformationValue( void ) const { return colorTransformationValue; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Category that contains the Action control features.
///
/// Category that contains the Action control features.
class ActionControl
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::ActionControl</b> object.
	explicit ActionControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			actionDeviceKey(),
			actionSelector(),
			actionGroupMask(),
			actionGroupKey()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam" );
		locator.bindComponent( actionDeviceKey, "ActionDeviceKey" );
		locator.bindComponent( actionSelector, "ActionSelector" );
		locator.bindComponent( actionGroupMask, "ActionGroupMask" );
		locator.bindComponent( actionGroupKey, "ActionGroupKey" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Provides the device key that allows the device to check the validity of action commands.
	///
	/// Provides the device key that allows the device to check the validity of action commands. The device internal assertion of an action signal is only authorized if the ActionDeviceKey and the action device key value in the protocol message are equal.
	PropertyI64 actionDeviceKey;
	/// \brief Selects to which Action Signal further Action settings apply.
	///
	/// Selects to which Action Signal further Action settings apply.
	PropertyI64 actionSelector;
	/// \brief Provides the mask that the device will use to validate the action on reception of the action protocol message.
	///
	/// Provides the mask that the device will use to validate the action on reception of the action protocol message.
	PropertyI64 actionGroupMask;
	/// \brief Provides the key that the device will use to validate the action on reception of the action protocol message.
	///
	/// Provides the key that the device will use to validate the action on reception of the action protocol message.
	PropertyI64 actionGroupKey;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getActionDeviceKey( void ) const { return actionDeviceKey; }
	PropertyI64 getActionSelector( void ) const { return actionSelector; }
	PropertyI64 getActionGroupMask( void ) const { return actionGroupMask; }
	PropertyI64 getActionGroupKey( void ) const { return actionGroupKey; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to control the devices Logic Gate Control parameters.
///
/// Contains features to control the devices Logic Gate Control parameters.
class mvLogicGateControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvLogicGateControl</b> object.
	explicit mvLogicGateControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvLogicGateANDSelector(),
			mvLogicGateANDSource1(),
			mvANDSource1Inverter(),
			mvLogicGateANDSource2(),
			mvANDSource2Inverter(),
			mvLogicGateORSelector(),
			mvLogicGateORSource1(),
			mvLogicGateORSource2(),
			mvLogicGateORSource3(),
			mvLogicGateORSource4(),
			mvLogicGateSourceSelector(),
			mvLogicGateSource(),
			mvLogicGateSourceInverter(),
			mvLogicGateANDTermSelector(),
			mvLogicGateANDTermSrc0(),
			mvLogicGateANDTermSrc1(),
			mvLogicGateORTermSelector(),
			mvLogicGateORTermSrc0(),
			mvLogicGateORTermSrc1()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvLogicGateControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvLogicGateANDSelector, "mvLogicGateANDSelector" );
		locator.bindComponent( mvLogicGateANDSource1, "mvLogicGateANDSource1" );
		locator.bindComponent( mvANDSource1Inverter, "mvANDSource1Inverter" );
		locator.bindComponent( mvLogicGateANDSource2, "mvLogicGateANDSource2" );
		locator.bindComponent( mvANDSource2Inverter, "mvANDSource2Inverter" );
		locator.bindComponent( mvLogicGateORSelector, "mvLogicGateORSelector" );
		locator.bindComponent( mvLogicGateORSource1, "mvLogicGateORSource1" );
		locator.bindComponent( mvLogicGateORSource2, "mvLogicGateORSource2" );
		locator.bindComponent( mvLogicGateORSource3, "mvLogicGateORSource3" );
		locator.bindComponent( mvLogicGateORSource4, "mvLogicGateORSource4" );
		locator.bindComponent( mvLogicGateSourceSelector, "mvLogicGateSourceSelector" );
		locator.bindComponent( mvLogicGateSource, "mvLogicGateSource" );
		locator.bindComponent( mvLogicGateSourceInverter, "mvLogicGateSourceInverter" );
		locator.bindComponent( mvLogicGateANDTermSelector, "mvLogicGateANDTermSelector" );
		locator.bindComponent( mvLogicGateANDTermSrc0, "mvLogicGateANDTermSrc0" );
		locator.bindComponent( mvLogicGateANDTermSrc1, "mvLogicGateANDTermSrc1" );
		locator.bindComponent( mvLogicGateORTermSelector, "mvLogicGateORTermSelector" );
		locator.bindComponent( mvLogicGateORTermSrc0, "mvLogicGateORTermSrc0" );
		locator.bindComponent( mvLogicGateORTermSrc1, "mvLogicGateORTermSrc1" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects the AND gate to configure.
	///
	/// This enumeration selects the AND gate to configure.
	PropertyI64 mvLogicGateANDSelector;
	/// \brief Selects the first input signal of the AND gate selected by mvLogicGateANDSelector.
	///
	/// This enumeration can be used to select the first input signal of the AND gate selected by mvLogicGateANDSelector.
	PropertyI64 mvLogicGateANDSource1;
	/// \brief Inverts the first input signal of the AND gate selected by mvLogicGateANDSelector
	///
	/// Inverts the first input signal of the AND gate selected by mvLogicGateANDSelector
	PropertyIBoolean mvANDSource1Inverter;
	/// \brief Selects the second input signal of the AND gate selected by mvLogicGateANDSelector.
	///
	/// This enumeration can be used to select the second input signal of the AND gate selected by mvLogicGateANDSelector.
	PropertyI64 mvLogicGateANDSource2;
	/// \brief Inverts the second input signal of the AND gate selected by mvLogicGateANDSelector
	///
	/// Inverts the second input signal of the AND gate selected by mvLogicGateANDSelector
	PropertyIBoolean mvANDSource2Inverter;
	/// \brief Selects the OR gate to configure.
	///
	/// This enumeration selects the OR gate to configure.
	PropertyI64 mvLogicGateORSelector;
	/// \brief Selects the first input signal of the OR gate selected by mvLogicGateORSelector.
	///
	/// This enumeration can be used to select the first input signal of the OR gate selected by mvLogicGateORSelector.
	PropertyI64 mvLogicGateORSource1;
	/// \brief Selects the second input signal of the OR gate selected by mvLogicGateORSelector.
	///
	/// This enumeration can be used to select the second input signal of the OR gate selected by mvLogicGateORSelector.
	PropertyI64 mvLogicGateORSource2;
	/// \brief Selects the third input signal of the OR gate selected by mvLogicGateORSelector.
	///
	/// This enumeration can be used to select the third input signal of the OR gate selected by mvLogicGateORSelector.
	PropertyI64 mvLogicGateORSource3;
	/// \brief Selects the fourth input signal of the OR gate selected by mvLogicGateORSelector.
	///
	/// This enumeration can be used to select the fourth input signal of the OR gate selected by mvLogicGateORSelector.
	PropertyI64 mvLogicGateORSource4;
	/// \brief Selects the LogicGateSource of the ANDORMatrix.
	///
	/// Selects the LogicGateSource of the ANDORMatrix.
	PropertyI64 mvLogicGateSourceSelector;
	/// \brief Selects the input signal of the ANDORMatrix selected by mvLogicGateSourceSelector.
	///
	/// Selects the input signal of the ANDORMatrix selected by mvLogicGateSourceSelector.
	PropertyI64 mvLogicGateSource;
	PropertyIBoolean mvLogicGateSourceInverter;
	/// \brief Selects the AND-term of the AND-OR-matrix.
	///
	/// Selects the AND-term of the AND-OR-matrix.
	PropertyI64 mvLogicGateANDTermSelector;
	/// \brief Selects the first input signal of the AND-term selected by mvLogicGateANDTermSelector.
	///
	/// Selects the first input signal of the AND-term selected by mvLogicGateANDTermSelector.
	PropertyI64 mvLogicGateANDTermSrc0;
	/// \brief Selects the second input signal of the AND-term selected by mvLogicGateANDTermSelector.
	///
	/// Selects the second input signal of the AND-term selected by mvLogicGateANDTermSelector.
	PropertyI64 mvLogicGateANDTermSrc1;
	/// \brief Selects the OR-term of the AND-OR-matrix.
	///
	/// Selects the OR-term of the AND-OR-matrix.
	PropertyI64 mvLogicGateORTermSelector;
	/// \brief Selects the first input signal of the ORTerm selected by mvLogicGateORTermSelector.
	///
	/// Selects the first input signal of the ORTerm selected by mvLogicGateORTermSelector.
	PropertyI64 mvLogicGateORTermSrc0;
	/// \brief Selects the second input signal of the ORTerm selected by mvLogicGateORTermSelector.
	///
	/// Selects the second input signal of the ORTerm selected by mvLogicGateORTermSelector.
	PropertyI64 mvLogicGateORTermSrc1;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getmvLogicGateANDSelector( void ) const { return mvLogicGateANDSelector; }
	PropertyI64 getmvLogicGateANDSource1( void ) const { return mvLogicGateANDSource1; }
	PropertyIBoolean getmvANDSource1Inverter( void ) const { return mvANDSource1Inverter; }
	PropertyI64 getmvLogicGateANDSource2( void ) const { return mvLogicGateANDSource2; }
	PropertyIBoolean getmvANDSource2Inverter( void ) const { return mvANDSource2Inverter; }
	PropertyI64 getmvLogicGateORSelector( void ) const { return mvLogicGateORSelector; }
	PropertyI64 getmvLogicGateORSource1( void ) const { return mvLogicGateORSource1; }
	PropertyI64 getmvLogicGateORSource2( void ) const { return mvLogicGateORSource2; }
	PropertyI64 getmvLogicGateORSource3( void ) const { return mvLogicGateORSource3; }
	PropertyI64 getmvLogicGateORSource4( void ) const { return mvLogicGateORSource4; }
	PropertyI64 getmvLogicGateSourceSelector( void ) const { return mvLogicGateSourceSelector; }
	PropertyI64 getmvLogicGateSource( void ) const { return mvLogicGateSource; }
	PropertyIBoolean getmvLogicGateSourceInverter( void ) const { return mvLogicGateSourceInverter; }
	PropertyI64 getmvLogicGateANDTermSelector( void ) const { return mvLogicGateANDTermSelector; }
	PropertyI64 getmvLogicGateANDTermSrc0( void ) const { return mvLogicGateANDTermSrc0; }
	PropertyI64 getmvLogicGateANDTermSrc1( void ) const { return mvLogicGateANDTermSrc1; }
	PropertyI64 getmvLogicGateORTermSelector( void ) const { return mvLogicGateORTermSelector; }
	PropertyI64 getmvLogicGateORTermSrc0( void ) const { return mvLogicGateORTermSrc0; }
	PropertyI64 getmvLogicGateORTermSrc1( void ) const { return mvLogicGateORTermSrc1; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to control the devices Current Control parameters.
///
/// Contains features to control the devices Current Control parameters.
class mvCurrentControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvCurrentControl</b> object.
	explicit mvCurrentControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvCurrentSelector(),
			mvCurrent()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvCurrentControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvCurrentSelector, "mvCurrentSelector" );
		locator.bindComponent( mvCurrent, "mvCurrent" );
	}
	PYTHON_ONLY(%immutable;)
	PropertyI64 mvCurrentSelector;
	PropertyI64 mvCurrent;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getmvCurrentSelector( void ) const { return mvCurrentSelector; }
	PropertyI64 getmvCurrent( void ) const { return mvCurrent; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to control the devices Flat Field Correction parameters.
///
/// Contains features to control the devices Flat Field Correction parameters.
class mvFFCControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvFFCControl</b> object.
	explicit mvFFCControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvFFCEnable(),
			mvFFCCalibrationImageCount(),
			mvFFCCalibrate()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvFFCControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvFFCEnable, "mvFFCEnable" );
		locator.bindComponent( mvFFCCalibrationImageCount, "mvFFCCalibrationImageCount" );
		locator.bindComponent( mvFFCCalibrate, "mvFFCCalibrate@i" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Enables the Flat Field Correction.
	///
	/// Enables the Flat Field Correction.
	PropertyIBoolean mvFFCEnable;
	/// \brief The number of images to use for the calculation of the correction image.
	///
	/// The number of images to use for the calculation of the correction image.
	PropertyI64 mvFFCCalibrationImageCount;
	/// \brief Starts the Calibration of the Flat Field Correction.
	///
	/// Starts the Calibration of the Flat Field Correction.
	Method mvFFCCalibrate;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyIBoolean getmvFFCEnable( void ) const { return mvFFCEnable; }
	PropertyI64 getmvFFCCalibrationImageCount( void ) const { return mvFFCCalibrationImageCount; }
	Method getmvFFCCalibrate( void ) const { return mvFFCCalibrate; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to control the frame averaging engine.
///
/// Contains features to control the frame averaging engine.
class mvFrameAverageControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvFrameAverageControl</b> object.
	explicit mvFrameAverageControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvFrameAverageEnable(),
			mvFrameAverageSlope()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvFrameAverageControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvFrameAverageEnable, "mvFrameAverageEnable" );
		locator.bindComponent( mvFrameAverageSlope, "mvFrameAverageSlope" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Enables the frame averaging engine.
	///
	/// Enables the frame averaging engine.
	PropertyIBoolean mvFrameAverageEnable;
	/// \brief The slope in full range of register.
	///
	/// The slope in full range of register.
	PropertyI64 mvFrameAverageSlope;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyIBoolean getmvFrameAverageEnable( void ) const { return mvFrameAverageEnable; }
	PropertyI64 getmvFrameAverageSlope( void ) const { return mvFrameAverageSlope; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to control the devices High Dynamic Range parameters.
///
/// Contains features to control the devices High Dynamic Range parameters.
class mvHDRControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvHDRControl</b> object.
	explicit mvHDRControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvHDREnable(),
			mvHDRPreset(),
			mvHDRSelector(),
			mvHDRVoltage1(),
			mvHDRVoltage2(),
			mvHDRExposure1(),
			mvHDRExposure2()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvHDRControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvHDREnable, "mvHDREnable" );
		locator.bindComponent( mvHDRPreset, "mvHDRPreset" );
		locator.bindComponent( mvHDRSelector, "mvHDRSelector" );
		locator.bindComponent( mvHDRVoltage1, "mvHDRVoltage1" );
		locator.bindComponent( mvHDRVoltage2, "mvHDRVoltage2" );
		locator.bindComponent( mvHDRExposure1, "mvHDRExposure1" );
		locator.bindComponent( mvHDRExposure2, "mvHDRExposure2" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Enables the High Dynamic Range Feature.
	///
	/// Enables the High Dynamic Range Feature.
	PropertyIBoolean mvHDREnable;
	/// \brief Selects the HDR parameter set.
	///
	/// Selects the HDR parameter set.
	PropertyI64 mvHDRPreset;
	/// \brief Selects the HDR parameter set to configure.
	///
	/// This enumeration selects the HDR parameter set to configure.
	PropertyI64 mvHDRSelector;
	/// \brief First HDR Voltage in mV.
	///
	/// First HDR Voltage in mV.
	PropertyI64 mvHDRVoltage1;
	/// \brief Second HDR Voltage in mV.
	///
	/// Second HDR Voltage in mV.
	PropertyI64 mvHDRVoltage2;
	/// \brief First HDR Exposure in ppm.
	///
	/// First HDR Exposure in ppm.
	PropertyI64 mvHDRExposure1;
	/// \brief Second HDR Exposure in ppm.
	///
	/// Second HDR Exposure in ppm.
	PropertyI64 mvHDRExposure2;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyIBoolean getmvHDREnable( void ) const { return mvHDREnable; }
	PropertyI64 getmvHDRPreset( void ) const { return mvHDRPreset; }
	PropertyI64 getmvHDRSelector( void ) const { return mvHDRSelector; }
	PropertyI64 getmvHDRVoltage1( void ) const { return mvHDRVoltage1; }
	PropertyI64 getmvHDRVoltage2( void ) const { return mvHDRVoltage2; }
	PropertyI64 getmvHDRExposure1( void ) const { return mvHDRExposure1; }
	PropertyI64 getmvHDRExposure2( void ) const { return mvHDRExposure2; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief Contains features to access the device internal SPI bus.
///
/// Contains features to access the device internal SPI bus.
class mvSPIControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvSPIControl</b> object.
	explicit mvSPIControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvSPIDeviceSelector(),
			mvSPIOperationSelector(),
			mvSPIOperationExecute(),
			mvSPIAccessBuffer(),
			mvSPIAccessLength()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvSPIControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvSPIDeviceSelector, "mvSPIDeviceSelector" );
		locator.bindComponent( mvSPIOperationSelector, "mvSPIOperationSelector" );
		locator.bindComponent( mvSPIOperationExecute, "mvSPIOperationExecute@i" );
		locator.bindComponent( mvSPIAccessBuffer, "mvSPIAccessBuffer" );
		locator.bindComponent( mvSPIAccessLength, "mvSPIAccessLength" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Selects the SPI device.
	///
	/// Selects the SPI device.
	PropertyI64 mvSPIDeviceSelector;
	/// \brief Selects the operation. Write: mvSPIAccessLength bytes are written to SPI device. Synchronuously read bytes are stored to internal buffer. Read: Reads mvSPIAccessLength from internal buffer. If mvSPIAccessLength > internal buffer size: Additional 'write zeros' will be done.
	///
	/// Selects the operation. Write: mvSPIAccessLength bytes are written to SPI device. Synchronuously read bytes are stored to internal buffer. Read: Reads mvSPIAccessLength from internal buffer. If mvSPIAccessLength > internal buffer size: Additional 'write zeros' will be done.
	PropertyI64 mvSPIOperationSelector;
	Method mvSPIOperationExecute;
	/// \brief Defines the intermediate access buffer that allows the exchange of data.
	///
	/// Defines the intermediate access buffer that allows the exchange of data.
	PropertyS mvSPIAccessBuffer;
	/// \brief Controls the length of the data.
	///
	/// Controls the length of the data.
	PropertyI64 mvSPIAccessLength;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getmvSPIDeviceSelector( void ) const { return mvSPIDeviceSelector; }
	PropertyI64 getmvSPIOperationSelector( void ) const { return mvSPIOperationSelector; }
	Method getmvSPIOperationExecute( void ) const { return mvSPIOperationExecute; }
	PropertyS getmvSPIAccessBuffer( void ) const { return mvSPIAccessBuffer; }
	PropertyI64 getmvSPIAccessLength( void ) const { return mvSPIAccessLength; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
class mvDACParams : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvDACParams</b> object.
	explicit mvDACParams(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvDACOUTA(),
			mvDACOUTB(),
			mvDACOUTC(),
			mvDACOUTD(),
			DACIndex(),
			DACValue(),
			DACValueAll(),
			mvErrorDetect0(),
			mvErrorDetect1()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvDACParams" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvDACOUTA, "mvDACOUTA" );
		locator.bindComponent( mvDACOUTB, "mvDACOUTB" );
		locator.bindComponent( mvDACOUTC, "mvDACOUTC" );
		locator.bindComponent( mvDACOUTD, "mvDACOUTD" );
		locator.bindComponent( DACIndex, "DACIndex" );
		locator.bindComponent( DACValue, "DACValue" );
		locator.bindComponent( DACValueAll, "DACValueAll" );
		locator.bindComponent( mvErrorDetect0, "mvErrorDetect0" );
		locator.bindComponent( mvErrorDetect1, "mvErrorDetect1" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Changes the current of the CCDs output signal (Tap1).
	///
	/// Changes the current of the CCDs output signal (Tap1).
	PropertyI64 mvDACOUTA;
	/// \brief Changes the current of the CCDs output signal (Tap2).
	///
	/// Changes the current of the CCDs output signal (Tap2).
	PropertyI64 mvDACOUTB;
	/// \brief Currently not used
	///
	/// Currently not used
	PropertyI64 mvDACOUTC;
	/// \brief Sets the sensor's VSUB voltage.
	///
	/// Sets the sensor's VSUB voltage.
	PropertyI64 mvDACOUTD;
	/// \brief Selects the digital to analog converter(DAC) DACValue will be written to
	///
	/// Selects the digital to analog converter(DAC) DACValue will be written to
	PropertyI64 DACIndex;
	/// \brief Register value in hex
	///
	/// Register value in hex
	PropertyI64 DACValue;
	/// \brief Write all DAcs with a single call.
	///
	/// Write all DAcs with a single call.
	PropertyS DACValueAll;
	/// \brief Result of the error detection mechanism for lamp0
	///
	/// Result of the error detection mechanism for lamp0
	PropertyI64 mvErrorDetect0;
	/// \brief Result of the error detection mechanism for lamp1
	///
	/// Result of the error detection mechanism for lamp1
	PropertyI64 mvErrorDetect1;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyI64 getmvDACOUTA( void ) const { return mvDACOUTA; }
	PropertyI64 getmvDACOUTB( void ) const { return mvDACOUTB; }
	PropertyI64 getmvDACOUTC( void ) const { return mvDACOUTC; }
	PropertyI64 getmvDACOUTD( void ) const { return mvDACOUTD; }
	PropertyI64 getDACIndex( void ) const { return DACIndex; }
	PropertyI64 getDACValue( void ) const { return DACValue; }
	PropertyS getDACValueAll( void ) const { return DACValueAll; }
	PropertyI64 getmvErrorDetect0( void ) const { return mvErrorDetect0; }
	PropertyI64 getmvErrorDetect1( void ) const { return mvErrorDetect1; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
class mvACCControl : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::mvACCControl</b> object.
	explicit mvACCControl(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			mvACCEnable(),
			mvXAxis(),
			mvYAxis(),
			mvZAxis()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenICam/mvACCControl" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( mvACCEnable, "mvACCEnable" );
		locator.bindComponent( mvXAxis, "mvXAxis" );
		locator.bindComponent( mvYAxis, "mvYAxis" );
		locator.bindComponent( mvZAxis, "mvZAxis" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Sets ACC in measurement mode.
	///
	/// Sets ACC in measurement mode.
	PropertyIBoolean mvACCEnable;
	/// \brief The postition in X-direction.
	///
	/// The postition in X-direction.
	PropertyI64 mvXAxis;
	/// \brief The postition in Y-direction.
	///
	/// The postition in Y-direction.
	PropertyI64 mvYAxis;
	/// \brief The postition in Z-direction.
	///
	/// The postition in Z-direction.
	PropertyI64 mvZAxis;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyIBoolean getmvACCEnable( void ) const { return mvACCEnable; }
	PropertyI64 getmvXAxis( void ) const { return mvXAxis; }
	PropertyI64 getmvYAxis( void ) const { return mvYAxis; }
	PropertyI64 getmvZAxis( void ) const { return mvZAxis; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief The System category includes items that belong to the system module of the transport layer.
///
/// The System category includes items that belong to the system module of the transport layer.
class SystemModule : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::SystemModule</b> object.
	explicit SystemModule(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev ) :
			mvIMPACT::acquire::ComponentCollection(INVALID_ID),
			TLVendorName(),
			TLModelName(),
			TLID(),
			TLVersion(),
			TLPath(),
			TLType(),
			interfaceUpdateList(),
			interfaceSelector(),
			interfaceID(),
			interfaceType(),
			gevInterfaceMACAddress(),
			gevInterfaceDefaultIPAddress(),
			gevInterfaceDefaultSubnetMask(),
			gevInterfaceDefaultGateway(),
			mvGevInterfaceMTU(),
			mvGevInterfaceLinkSpeed(),
			genTLVersionMajor(),
			genTLVersionMinor(),
			gevVersionMajor(),
			gevVersionMinor(),
			mvGevChannelDummyPacketSendEnable(),
			mvGevChannelDummyPacketSendInterval()
	{
		mvIMPACT::acquire::ComponentLocator locator(pDev->deviceDriverFeatureList());
		locator.bindSearchBase( locator.searchbase_id(), "GenTL/System" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( TLVendorName, "TLVendorName" );
		locator.bindComponent( TLModelName, "TLModelName" );
		locator.bindComponent( TLID, "TLID" );
		locator.bindComponent( TLVersion, "TLVersion" );
		locator.bindComponent( TLPath, "TLPath" );
		locator.bindComponent( TLType, "TLType" );
		locator.bindComponent( interfaceUpdateList, "InterfaceUpdateList@i" );
		locator.bindComponent( interfaceSelector, "InterfaceSelector" );
		locator.bindComponent( interfaceID, "InterfaceID" );
		locator.bindComponent( interfaceType, "InterfaceType" );
		locator.bindComponent( gevInterfaceMACAddress, "GevInterfaceMACAddress" );
		locator.bindComponent( gevInterfaceDefaultIPAddress, "GevInterfaceDefaultIPAddress" );
		locator.bindComponent( gevInterfaceDefaultSubnetMask, "GevInterfaceDefaultSubnetMask" );
		locator.bindComponent( gevInterfaceDefaultGateway, "GevInterfaceDefaultGateway" );
		locator.bindComponent( mvGevInterfaceMTU, "mvGevInterfaceMTU" );
		locator.bindComponent( mvGevInterfaceLinkSpeed, "mvGevInterfaceLinkSpeed" );
		locator.bindComponent( genTLVersionMajor, "GenTLVersionMajor" );
		locator.bindComponent( genTLVersionMinor, "GenTLVersionMinor" );
		locator.bindComponent( gevVersionMajor, "GevVersionMajor" );
		locator.bindComponent( gevVersionMinor, "GevVersionMinor" );
		locator.bindComponent( mvGevChannelDummyPacketSendEnable, "mvGevChannelDummyPacketSendEnable" );
		locator.bindComponent( mvGevChannelDummyPacketSendInterval, "mvGevChannelDummyPacketSendInterval" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Indicates the name of the transport layer vendor.
	///
	/// This is a read only element. It is a string that indicates the name of the transport layer vendor.
	PropertyS TLVendorName;
	/// \brief Indicates the name of the transport layer Model.
	///
	/// This is a read only element. It is a string that indicates the name of the transport layer Model to distinguish different kinds of GenTL Producer implementations from one vendor.
	PropertyS TLModelName;
	/// \brief Indicates the ID of the transport layer.
	///
	/// This is a read only element. It is a string that indicates the ID of the transport layer.
	PropertyS TLID;
	/// \brief Indicates a vendor specific version string for this transport layer.
	///
	/// This is a read only element. It is a string that indicates a vendor specific version string for this transport layer.
	PropertyS TLVersion;
	/// \brief Indicates the full path to the GenTL Producer driver including name and extension.
	///
	/// This is a read only element. It is a string that indicates the full path to the GenTL Producer driver including name and extension.
	PropertyS TLPath;
	/// \brief Identifies the transport layer technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates the transport layer technology of the GenTL Producer implementation.
	PropertyI64 TLType;
	/// \brief Updates the internal interface list.
	///
	/// This command updates the internal interface list of this transport layer.
	Method interfaceUpdateList;
	/// \brief Selector for the different GenTL Producer interfaces.
	///
	/// Selector for the different GenTL Producer interfaces. Selector for the different GenTL Producer interfaces. This interface list only changes on execution of InterfaceUpdateList. The selector is 0-based in order to match the index of the C interface.
	PropertyI64 interfaceSelector;
	/// \brief GenTL producer wide unique identifier of the selected interface.
	///
	/// This is a read only element. It is a string that indicates a GenTL producer wide unique identifier of the selected interface.
	PropertyS interfaceID;
	/// \brief Identifies the interfaces technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates interfaces technology of the GenTL Producer implementation.
	PropertyI64 interfaceType;
	/// \brief Indicates the 48-bit MAC address of the selected interface.
	///
	/// This is a read only element. It indicates the 48-bit MAC address of the selected interface.
	PropertyI64 gevInterfaceMACAddress;
	/// \brief Indicates the IP address of the first subnet of the selected interface.
	///
	/// This is a read only element. It indicates the IP address of the first subnet of the selected interface.
	PropertyI64 gevInterfaceDefaultIPAddress;
	/// \brief Indicates the subnet mask of the first subnet of the selected interface.
	///
	/// This is a read only element. It indicates the subnet mask of the first subnet of the selected interface.
	PropertyI64 gevInterfaceDefaultSubnetMask;
	/// \brief Indicates the default gateway of the first subnet of the selected interface.
	///
	/// This is a read only element. It indicates the default gateway of the first subnet of the selected interface.
	PropertyI64 gevInterfaceDefaultGateway;
	/// \brief Indicates the MTU of the selected interface.
	///
	/// This is a read only element. It indicates the MTU(Maximum Transmission Unit) of the selected interface.
	PropertyI64 mvGevInterfaceMTU;
	/// \brief Indicates the link speed of this interface.
	///
	/// This is a read only element. It indicates the link speed(in Mbits per second) of this interface.
	PropertyI64 mvGevInterfaceLinkSpeed;
	/// \brief Defines the major version number of the GenTL specification the GenTL Producer implementation complies with.
	///
	/// This is a read only element. It defines the major version number of the GenTL specification the GenTL Producer implementation complies with.
	PropertyI64 genTLVersionMajor;
	/// \brief Defines the minor version number of the GenTL specification the GenTL Producer implementation complies with.
	///
	/// This is a read only element. It defines the minor version number of the GenTL specification the GenTL Producer implementation complies with.
	PropertyI64 genTLVersionMinor;
	/// \brief Major version of the specification.
	///
	/// Major version of the specification.
	PropertyI64 gevVersionMajor;
	/// \brief Minor version of the specification.
	///
	/// Minor version of the specification.
	PropertyI64 gevVersionMinor;
	/// \brief Enables or disables the periodical sending of dummy packets to a stream or message channel source port of a GigE Vision device.
	///
	/// Enables or disables the periodical sending of dummy packets to a stream or message channel source port of a GigE Vision device. This might be useful to overcome firewall related problems when working with network devices.
	PropertyIBoolean mvGevChannelDummyPacketSendEnable;
	/// \brief Defines the period(in ms) for sending dummy packets to a stream or message channel source port of a GigE Vision device.
	///
	/// Defines the period(in ms) for sending dummy packets to a stream or message channel source port of a GigE Vision device. This might be useful to overcome firewall related problems when working with network devices.
	PropertyI64 mvGevChannelDummyPacketSendInterval;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyS getTLVendorName( void ) const { return TLVendorName; }
	PropertyS getTLModelName( void ) const { return TLModelName; }
	PropertyS getTLID( void ) const { return TLID; }
	PropertyS getTLVersion( void ) const { return TLVersion; }
	PropertyS getTLPath( void ) const { return TLPath; }
	PropertyI64 getTLType( void ) const { return TLType; }
	Method getInterfaceUpdateList( void ) const { return interfaceUpdateList; }
	PropertyI64 getInterfaceSelector( void ) const { return interfaceSelector; }
	PropertyS getInterfaceID( void ) const { return interfaceID; }
	PropertyI64 getInterfaceType( void ) const { return interfaceType; }
	PropertyI64 getGevInterfaceMACAddress( void ) const { return gevInterfaceMACAddress; }
	PropertyI64 getGevInterfaceDefaultIPAddress( void ) const { return gevInterfaceDefaultIPAddress; }
	PropertyI64 getGevInterfaceDefaultSubnetMask( void ) const { return gevInterfaceDefaultSubnetMask; }
	PropertyI64 getGevInterfaceDefaultGateway( void ) const { return gevInterfaceDefaultGateway; }
	PropertyI64 getmvGevInterfaceMTU( void ) const { return mvGevInterfaceMTU; }
	PropertyI64 getmvGevInterfaceLinkSpeed( void ) const { return mvGevInterfaceLinkSpeed; }
	PropertyI64 getGenTLVersionMajor( void ) const { return genTLVersionMajor; }
	PropertyI64 getGenTLVersionMinor( void ) const { return genTLVersionMinor; }
	PropertyI64 getGevVersionMajor( void ) const { return gevVersionMajor; }
	PropertyI64 getGevVersionMinor( void ) const { return gevVersionMinor; }
	PropertyIBoolean getmvGevChannelDummyPacketSendEnable( void ) const { return mvGevChannelDummyPacketSendEnable; }
	PropertyI64 getmvGevChannelDummyPacketSendInterval( void ) const { return mvGevChannelDummyPacketSendInterval; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief The Interface category includes items that belong to the interface module of the transport layer.
///
/// The Interface category includes items that belong to the interface module of the transport layer.
class InterfaceModule : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::InterfaceModule</b> object.
	explicit InterfaceModule(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The \a index of the instance this object shall be created for. Passing an invalid index will raise an exception.
		int index ) :
			mvIMPACT::acquire::ComponentCollection(INVALID_ID),
			interfaceID(),
			interfaceType(),
			gevInterfaceGatewaySelector(),
			gevInterfaceGateway(),
			gevInterfaceMACAddress(),
			gevInterfaceSubnetSelector(),
			gevInterfaceSubnetIPAddress(),
			gevInterfaceSubnetMask(),
			mvGevInterfaceMTU(),
			mvGevInterfaceLinkSpeed(),
			mvGevAdvancedDeviceDiscoveryEnable(),
			deviceUpdateList(),
			deviceSelector(),
			deviceID(),
			deviceType(),
			deviceVendorName(),
			deviceModelName(),
			deviceAccessStatus(),
			mvDevicePrimaryApplicationSwitchoverSupported(),
			mvDevicePrimaryApplicationSwitchoverEnable(),
			mvDevicePrimaryApplicationSwitchoverKey(),
			mvDeviceNetworkInterfaceCount(),
			gevDeviceIPAddress(),
			gevDeviceSubnetMask(),
			gevDeviceMACAddress()
	{
		mvIMPACT::acquire::ComponentLocator locator(pDev->deviceDriverFeatureList());
		locator.bindSearchBase( locator.searchbase_id(), "GenTL/Interfaces" );
		std::ostringstream oss;
		oss << "Interface" << index;
		locator = mvIMPACT::acquire::ComponentLocator(locator.findComponent( oss.str() ));
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( interfaceID, "InterfaceID" );
		locator.bindComponent( interfaceType, "InterfaceType" );
		locator.bindComponent( gevInterfaceGatewaySelector, "GevInterfaceGatewaySelector" );
		locator.bindComponent( gevInterfaceGateway, "GevInterfaceGateway" );
		locator.bindComponent( gevInterfaceMACAddress, "GevInterfaceMACAddress" );
		locator.bindComponent( gevInterfaceSubnetSelector, "GevInterfaceSubnetSelector" );
		locator.bindComponent( gevInterfaceSubnetIPAddress, "GevInterfaceSubnetIPAddress" );
		locator.bindComponent( gevInterfaceSubnetMask, "GevInterfaceSubnetMask" );
		locator.bindComponent( mvGevInterfaceMTU, "mvGevInterfaceMTU" );
		locator.bindComponent( mvGevInterfaceLinkSpeed, "mvGevInterfaceLinkSpeed" );
		locator.bindComponent( mvGevAdvancedDeviceDiscoveryEnable, "mvGevAdvancedDeviceDiscoveryEnable" );
		locator.bindComponent( deviceUpdateList, "DeviceUpdateList@i" );
		locator.bindComponent( deviceSelector, "DeviceSelector" );
		locator.bindComponent( deviceID, "DeviceID" );
		locator.bindComponent( deviceType, "DeviceType" );
		locator.bindComponent( deviceVendorName, "DeviceVendorName" );
		locator.bindComponent( deviceModelName, "DeviceModelName" );
		locator.bindComponent( deviceAccessStatus, "DeviceAccessStatus" );
		locator.bindComponent( mvDevicePrimaryApplicationSwitchoverSupported, "mvDevicePrimaryApplicationSwitchoverSupported" );
		locator.bindComponent( mvDevicePrimaryApplicationSwitchoverEnable, "mvDevicePrimaryApplicationSwitchoverEnable" );
		locator.bindComponent( mvDevicePrimaryApplicationSwitchoverKey, "mvDevicePrimaryApplicationSwitchoverKey" );
		locator.bindComponent( mvDeviceNetworkInterfaceCount, "mvDeviceNetworkInterfaceCount" );
		locator.bindComponent( gevDeviceIPAddress, "GevDeviceIPAddress" );
		locator.bindComponent( gevDeviceSubnetMask, "GevDeviceSubnetMask" );
		locator.bindComponent( gevDeviceMACAddress, "GevDeviceMACAddress" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief GenTL producer wide unique identifier of the selected interface.
	///
	/// This is a read only element. It is a string that indicates a GenTL producer wide unique identifier of the selected interface.
	PropertyS interfaceID;
	/// \brief Identifies the interfaces technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates interfaces technology of the GenTL Producer implementation.
	PropertyI64 interfaceType;
	/// \brief Selector for the different gateway entries for this interface.
	///
	/// Selector for the different gateway entries for this interface. The selector is 0-based in order to match the index of the C interface.
	PropertyI64 gevInterfaceGatewaySelector;
	/// \brief Indicates the IP address of the selected gateway entry of this interface.
	///
	/// This is a read only element. It indicates the IP address of the selected gateway entry of this interface.
	PropertyI64 gevInterfaceGateway;
	/// \brief Indicates the 48-bit MAC address of the selected interface.
	///
	/// This is a read only element. It indicates the 48-bit MAC address of the selected interface.
	PropertyI64 gevInterfaceMACAddress;
	/// \brief Selector for the different subnet entries for this interface.
	///
	/// Selector for the different subnet entries for this interface. The selector is 0-based in order to match the index of the C interface.
	PropertyI64 gevInterfaceSubnetSelector;
	/// \brief Indicates the IP address of the selected subnet entry of this interface.
	///
	/// This is a read only element. It indicates the IP address of the selected subnet entry of this interface.
	PropertyI64 gevInterfaceSubnetIPAddress;
	/// \brief Indicates the subnet mask of the selected subnet entry of this interface.
	///
	/// This is a read only element. It indicates the subnet mask of the selected subnet entry of this interface.
	PropertyI64 gevInterfaceSubnetMask;
	/// \brief Indicates the MTU of this interface.
	///
	/// This is a read only element. It indicates the MTU(Maximum Transmission Unit) of this interface.
	PropertyI64 mvGevInterfaceMTU;
	/// \brief Indicates the link speed of this interface.
	///
	/// This is a read only element. It indicates the link speed(in Mbits per second) of this interface.
	PropertyI64 mvGevInterfaceLinkSpeed;
	/// \brief Enables or disables the advanced device discovery.
	///
	/// Enables or disables the advanced device discovery. When enabled also devices, which currently use an incorrect IP configuration for the network they are connected to might be detectable. However depending on the operating system this may result in devices showing up on interfaces to which they are not physically connected.
	PropertyIBoolean mvGevAdvancedDeviceDiscoveryEnable;
	/// \brief Updates the internal device list of this interface.
	///
	/// This command updates the internal device list of this interface.
	Method deviceUpdateList;
	/// \brief Selector for the different devices on this interface.
	///
	/// Selector for the different devices on this interface. The limits of this feature might change upon execution of DeviceUpdateList.
	PropertyI64 deviceSelector;
	/// \brief Device Identifier (serial number).
	///
	/// Device Identifier (serial number).
	PropertyS deviceID;
	/// \brief Identifies the device technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates device technology of the GenTL Producer implementation.
	PropertyI64 deviceType;
	/// \brief Name of the manufacturer of the device.
	///
	/// Name of the manufacturer of the device.
	PropertyS deviceVendorName;
	/// \brief Model of the device.
	///
	/// Model of the device.
	PropertyS deviceModelName;
	/// \brief Indicates the current access status for the device.
	///
	/// This is a read only feature. This enumeration provides a value that indicates the current access status for the device.
	PropertyI64 deviceAccessStatus;
	/// \brief Reports the availability of the primary application switchover feature.
	///
	/// Reports the availability of the primary application switchover feature.
	PropertyIBoolean mvDevicePrimaryApplicationSwitchoverSupported;
	/// \brief Enables or disables primary application switchover.
	///
	/// Enables or disables primary application switchover.
	PropertyIBoolean mvDevicePrimaryApplicationSwitchoverEnable;
	/// \brief Controls the key to use to authenticate primary application switchover requests.
	///
	/// Controls the key to use to authenticate primary application switchover requests. If the device to take over has 'mvDevicePrimaryApplicationSwitchoverEnable' set to true and this key matches the devices internal key control access will be granted.
	PropertyI64 mvDevicePrimaryApplicationSwitchoverKey;
	/// \brief The number of physical network interfaces supported by this device.
	///
	/// This is an integer feature. It contains the number of physical network interfaces supported by this device.
	PropertyI64 mvDeviceNetworkInterfaceCount;
	/// \brief Indicates the current IP address of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the current IP address of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceIPAddress;
	/// \brief Indicates the current subnet mask of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the current subnet mask of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceSubnetMask;
	/// \brief Indicates the 48-bit MAC address of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the 48-bit MAC address of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceMACAddress;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyS getInterfaceID( void ) const { return interfaceID; }
	PropertyI64 getInterfaceType( void ) const { return interfaceType; }
	PropertyI64 getGevInterfaceGatewaySelector( void ) const { return gevInterfaceGatewaySelector; }
	PropertyI64 getGevInterfaceGateway( void ) const { return gevInterfaceGateway; }
	PropertyI64 getGevInterfaceMACAddress( void ) const { return gevInterfaceMACAddress; }
	PropertyI64 getGevInterfaceSubnetSelector( void ) const { return gevInterfaceSubnetSelector; }
	PropertyI64 getGevInterfaceSubnetIPAddress( void ) const { return gevInterfaceSubnetIPAddress; }
	PropertyI64 getGevInterfaceSubnetMask( void ) const { return gevInterfaceSubnetMask; }
	PropertyI64 getmvGevInterfaceMTU( void ) const { return mvGevInterfaceMTU; }
	PropertyI64 getmvGevInterfaceLinkSpeed( void ) const { return mvGevInterfaceLinkSpeed; }
	PropertyIBoolean getmvGevAdvancedDeviceDiscoveryEnable( void ) const { return mvGevAdvancedDeviceDiscoveryEnable; }
	Method getDeviceUpdateList( void ) const { return deviceUpdateList; }
	PropertyI64 getDeviceSelector( void ) const { return deviceSelector; }
	PropertyS getDeviceID( void ) const { return deviceID; }
	PropertyI64 getDeviceType( void ) const { return deviceType; }
	PropertyS getDeviceVendorName( void ) const { return deviceVendorName; }
	PropertyS getDeviceModelName( void ) const { return deviceModelName; }
	PropertyI64 getDeviceAccessStatus( void ) const { return deviceAccessStatus; }
	PropertyIBoolean getmvDevicePrimaryApplicationSwitchoverSupported( void ) const { return mvDevicePrimaryApplicationSwitchoverSupported; }
	PropertyIBoolean getmvDevicePrimaryApplicationSwitchoverEnable( void ) const { return mvDevicePrimaryApplicationSwitchoverEnable; }
	PropertyI64 getmvDevicePrimaryApplicationSwitchoverKey( void ) const { return mvDevicePrimaryApplicationSwitchoverKey; }
	PropertyI64 getmvDeviceNetworkInterfaceCount( void ) const { return mvDeviceNetworkInterfaceCount; }
	PropertyI64 getGevDeviceIPAddress( void ) const { return gevDeviceIPAddress; }
	PropertyI64 getGevDeviceSubnetMask( void ) const { return gevDeviceSubnetMask; }
	PropertyI64 getGevDeviceMACAddress( void ) const { return gevDeviceMACAddress; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief The Device category includes items that belong to the device module of the transport layer.
///
/// The Device category includes items that belong to the device module of the transport layer.
class DeviceModule : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::DeviceModule</b> object.
	explicit DeviceModule(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			deviceID(),
			deviceVendorName(),
			deviceModelName(),
			deviceType(),
			gevDeviceIPAddress(),
			gevDeviceSubnetMask(),
			gevDeviceMACAddress(),
			gevDeviceGateway(),
			deviceEndianessMechanism(),
			streamSelector(),
			streamID()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenTL/Device" );
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( deviceID, "DeviceID" );
		locator.bindComponent( deviceVendorName, "DeviceVendorName" );
		locator.bindComponent( deviceModelName, "DeviceModelName" );
		locator.bindComponent( deviceType, "DeviceType" );
		locator.bindComponent( gevDeviceIPAddress, "GevDeviceIPAddress" );
		locator.bindComponent( gevDeviceSubnetMask, "GevDeviceSubnetMask" );
		locator.bindComponent( gevDeviceMACAddress, "GevDeviceMACAddress" );
		locator.bindComponent( gevDeviceGateway, "GevDeviceGateway" );
		locator.bindComponent( deviceEndianessMechanism, "DeviceEndianessMechanism" );
		locator.bindComponent( streamSelector, "StreamSelector" );
		locator.bindComponent( streamID, "StreamID" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Device Identifier (serial number).
	///
	/// Device Identifier (serial number).
	PropertyS deviceID;
	/// \brief Name of the manufacturer of the device.
	///
	/// Name of the manufacturer of the device.
	PropertyS deviceVendorName;
	/// \brief Model of the device.
	///
	/// Model of the device.
	PropertyS deviceModelName;
	/// \brief Identifies the device technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates device technology of the GenTL Producer implementation.
	PropertyI64 deviceType;
	/// \brief Indicates the current IP address of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the current IP address of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceIPAddress;
	/// \brief Indicates the current subnet mask of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the current subnet mask of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceSubnetMask;
	/// \brief Indicates the 48-bit MAC address of the GVCP interface of the selected remote device.
	///
	/// This is a read only element. It indicates the 48-bit MAC address of the GVCP interface of the selected remote device.
	PropertyI64 gevDeviceMACAddress;
	/// \brief Indicates the current gateway of the GVCP interface of the remote device.
	///
	/// This is a read only element. It indicates the current gateway of the GVCP interface of the remote device.
	PropertyI64 gevDeviceGateway;
	/// \brief Identifies the endianess mode to be used for this device.
	///
	/// This is a read only feature. This enumeration provides a value that indicates the endianess mode to be used for this device.
	PropertyI64 deviceEndianessMechanism;
	/// \brief Selects the stream channel.
	///
	/// Selects the stream channel.
	PropertyI64 streamSelector;
	/// \brief Device wide unique ID of the selected stream.
	///
	/// This is a read only element. It is a string that indicates a device wide unique identifier of the selected stream.
	PropertyS streamID;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyS getDeviceID( void ) const { return deviceID; }
	PropertyS getDeviceVendorName( void ) const { return deviceVendorName; }
	PropertyS getDeviceModelName( void ) const { return deviceModelName; }
	PropertyI64 getDeviceType( void ) const { return deviceType; }
	PropertyI64 getGevDeviceIPAddress( void ) const { return gevDeviceIPAddress; }
	PropertyI64 getGevDeviceSubnetMask( void ) const { return gevDeviceSubnetMask; }
	PropertyI64 getGevDeviceMACAddress( void ) const { return gevDeviceMACAddress; }
	PropertyI64 getGevDeviceGateway( void ) const { return gevDeviceGateway; }
	PropertyI64 getDeviceEndianessMechanism( void ) const { return deviceEndianessMechanism; }
	PropertyI64 getStreamSelector( void ) const { return streamSelector; }
	PropertyS getStreamID( void ) const { return streamID; }
#endif // #ifdef DOTNET_ONLY_CODE
};

//-----------------------------------------------------------------------------
/// \brief The DataStream category includes items that belong to the data stream module of the transport layer.
///
/// The DataStream category includes items that belong to the data stream module of the transport layer.
class DataStreamModule : public mvIMPACT::acquire::ComponentCollection
//-----------------------------------------------------------------------------
{
public:
	/// \brief Constructs a new <b>mvIMPACT::acquire::GenICam::DataStreamModule</b> object.
	explicit DataStreamModule(
		/// A pointer to a <b>mvIMPACT::acquire::Device</b> object obtained from a <b>mvIMPACT::acquire::DeviceManager</b> object.
		mvIMPACT::acquire::Device* pDev,
		/// The \a index of the instance this object shall be created for. Passing an invalid index will raise an exception.
		int index,
		/// The name of the driver internal setting to access with this instance.
		/// A list of valid setting names can be obtained by a call to
		/// <b>mvIMPACT::acquire::FunctionInterface::getAvailableSettings</b>, new
		/// settings can be created with the function
		/// <b>mvIMPACT::acquire::FunctionInterface::createSetting</b>
		const std::string& settingName = "Base" ) :
			mvIMPACT::acquire::ComponentCollection(pDev),
			streamID(),
			streamAnnouncedBufferCount(),
			mvStreamAnnounceBufferMaximum(),
			streamAcquisitionModeSelector(),
			streamAnnounceBufferMinimum(),
			streamType(),
			mvStreamDriverTechnology(),
			mvResendActive(),
			mvResendMode(),
			mvResendBatchingActive(),
			mvResendCaptureWindowSize(),
			mvResendThreshold(),
			mvResendRequestMax(),
			mvResendRequestCredits(),
			mvResendResponseTimeout(),
			mvResendsPerTimeout(),
			mvResendFeaturesLocked()
	{
		mvIMPACT::acquire::DeviceComponentLocator locator(pDev, mvIMPACT::acquire::dltSetting, settingName);
		locator.bindSearchBase( locator.searchbase_id(), "Camera/GenTL/DataStreams" );
		std::ostringstream oss;
		oss << "Stream" << index;
		locator = mvIMPACT::acquire::DeviceComponentLocator(locator.findComponent( oss.str() ));
		m_hRoot = locator.searchbase_id();
		locator.bindComponent( streamID, "StreamID" );
		locator.bindComponent( streamAnnouncedBufferCount, "StreamAnnouncedBufferCount" );
		locator.bindComponent( mvStreamAnnounceBufferMaximum, "mvStreamAnnounceBufferMaximum" );
		locator.bindComponent( streamAcquisitionModeSelector, "StreamAcquisitionModeSelector" );
		locator.bindComponent( streamAnnounceBufferMinimum, "StreamAnnounceBufferMinimum" );
		locator.bindComponent( streamType, "StreamType" );
		locator.bindComponent( mvStreamDriverTechnology, "mvStreamDriverTechnology" );
		locator.bindComponent( mvResendActive, "mvResendActive" );
		locator.bindComponent( mvResendMode, "mvResendMode" );
		locator.bindComponent( mvResendBatchingActive, "mvResendBatchingActive" );
		locator.bindComponent( mvResendCaptureWindowSize, "mvResendCaptureWindowSize" );
		locator.bindComponent( mvResendThreshold, "mvResendThreshold" );
		locator.bindComponent( mvResendRequestMax, "mvResendRequestMax" );
		locator.bindComponent( mvResendRequestCredits, "mvResendRequestCredits" );
		locator.bindComponent( mvResendResponseTimeout, "mvResendResponseTimeout" );
		locator.bindComponent( mvResendsPerTimeout, "mvResendsPerTimeout" );
		locator.bindComponent( mvResendFeaturesLocked, "mvResendFeaturesLocked" );
	}
	PYTHON_ONLY(%immutable;)
	/// \brief Device wide unique ID of the selected stream.
	///
	/// This is a read only element. It is a string that indicates a device wide unique identifier of the selected stream.
	PropertyS streamID;
	/// \brief Number of announced (known) buffers on this stream. This value is volatile. It may change if additional buffers are announced and/or buffers are revoked by the GenTL Consumer.
	///
	/// This is a read-only feature. It indicates the number of announced (known) buffers on this stream. This value is volatile. It may change if additional buffers are announced and/or buffers are revoked by the GenTL Consumer.
	PropertyI64 streamAnnouncedBufferCount;
	/// \brief Maximal number of buffers to announce to enable selected acquisition mode.
	///
	/// This feature indicates the maximal number of buffers to announce to enable selected acquisition mode.
	PropertyI64 mvStreamAnnounceBufferMaximum;
	/// \brief Allows to select the acquisition mode for the stream.
	///
	/// This enumeration allows the selection of the acquisition mode for the stream.
	PropertyI64 streamAcquisitionModeSelector;
	/// \brief Minimal number of buffers to announce to enable selected acquisition mode.
	///
	/// This feature indicates the minimal number of buffers to announce to enable selected acquisition mode.
	PropertyI64 streamAnnounceBufferMinimum;
	/// \brief Identifies the stream technology of the GenTL Producer implementation.
	///
	/// This is a read only feature. This enumeration provides a value that indicates stream technology of the GenTL Producer implementation.
	PropertyI64 streamType;
	/// \brief The underlying driver technology used by this stream.
	///
	/// This is a read only element. It is a string that contains the underlying driver technology used by this stream.
	PropertyS mvStreamDriverTechnology;
	/// \brief This feature controls if the stream will issue packet resend requests.
	///
	/// This feature controls if the stream will issue packet resend requests.
	PropertyIBoolean mvResendActive;
	/// \brief Indicates the mode the internal resend algorithm is working in.
	///
	/// This feature indicates the mode the internal resend algorithm is working in.
	PropertyI64 mvResendMode;
	/// \brief This feature controls if the stream will issue batched packet resend requests if it detects several consecutive missing packets.
	///
	/// This feature controls if the stream will issue batched packet resend requests if it detects several consecutive missing packets.
	PropertyIBoolean mvResendBatchingActive;
	/// \brief Indicates the width of the capture window.
	///
	/// This feature indicates the width of the capture window in packets.
	PropertyI64 mvResendCaptureWindowSize;
	/// \brief Indicates the resend threshold within the capture window.
	///
	/// This feature indicates the resend threshold within the capture window. If current packet ID and first missing packet ID are mvResendThreshold IDs apart the stream will issue a resend request.
	PropertyI64 mvResendThreshold;
	/// \brief Indicates the maximum number of resend requests per packet to send to the device until the packet is considered as lost.
	///
	/// This feature indicates the maximum number of resend requests per packet to send to the device until the packet is considered as lost.
	PropertyI64 mvResendRequestMax;
	/// \brief Indicates the maximum number of resend requests in percent of payload packets per buffer to send to the device until the packet is considered as lost.
	///
	/// This feature indicates the maximum number of resend requests in percent of payload packets per buffer to send to the device until the packet is considered as lost.
	PropertyF mvResendRequestCredits;
	/// \brief This feature controls the resend response timeout in milli-seconds.
	///
	/// This float value sets the resend response timeout in milli-seconds. Whenever a requested packet does not arive wirthin this timer period it will be requested again until mvResendRequestMax resend requests for this packet have been issued.
	PropertyI64 mvResendResponseTimeout;
	/// \brief Indicates the number of packets to be requested whenever the resend response timeout elapses.
	///
	/// This feature indicates the number of packets to be requested whenever the resend response timeout elapses.
	PropertyI64 mvResendsPerTimeout;
	PropertyI64 mvResendFeaturesLocked;
	PYTHON_ONLY(%mutable;)
#ifdef DOTNET_ONLY_CODE
	PropertyS getStreamID( void ) const { return streamID; }
	PropertyI64 getStreamAnnouncedBufferCount( void ) const { return streamAnnouncedBufferCount; }
	PropertyI64 getmvStreamAnnounceBufferMaximum( void ) const { return mvStreamAnnounceBufferMaximum; }
	PropertyI64 getStreamAcquisitionModeSelector( void ) const { return streamAcquisitionModeSelector; }
	PropertyI64 getStreamAnnounceBufferMinimum( void ) const { return streamAnnounceBufferMinimum; }
	PropertyI64 getStreamType( void ) const { return streamType; }
	PropertyS getmvStreamDriverTechnology( void ) const { return mvStreamDriverTechnology; }
	PropertyIBoolean getmvResendActive( void ) const { return mvResendActive; }
	PropertyI64 getmvResendMode( void ) const { return mvResendMode; }
	PropertyIBoolean getmvResendBatchingActive( void ) const { return mvResendBatchingActive; }
	PropertyI64 getmvResendCaptureWindowSize( void ) const { return mvResendCaptureWindowSize; }
	PropertyI64 getmvResendThreshold( void ) const { return mvResendThreshold; }
	PropertyI64 getmvResendRequestMax( void ) const { return mvResendRequestMax; }
	PropertyF getmvResendRequestCredits( void ) const { return mvResendRequestCredits; }
	PropertyI64 getmvResendResponseTimeout( void ) const { return mvResendResponseTimeout; }
	PropertyI64 getmvResendsPerTimeout( void ) const { return mvResendsPerTimeout; }
	PropertyI64 getmvResendFeaturesLocked( void ) const { return mvResendFeaturesLocked; }
#endif // #ifdef DOTNET_ONLY_CODE
};

/// @}

		} // namespace GenICam
	} // namespace acquire
} // namespace mvIMPACT

#endif //mvIMPACT_acquire_GenICam_CPP_autogen_h
