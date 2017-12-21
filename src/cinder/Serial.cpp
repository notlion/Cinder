/*
 Copyright (c) 2009, The Barbarian Group
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#include "cinder/Serial.h"
#include "cinder/Timer.h"
#include "cinder/Thread.h"
#include "cinder/Utilities.h"
#include "cinder/Unicode.h"

#include <string>
#include <iostream>
#include <fcntl.h>

#if defined( CINDER_POSIX )
	#include <termios.h>
	#include <sys/ioctl.h>
	#include <getopt.h>
	#include <dirent.h>
#elif defined( CINDER_MSW )
	#include <windows.h>
	#include <setupapi.h>
	#pragma comment(lib, "setupapi.lib")
#endif

using namespace std;

namespace cinder {

bool							Serial::sDevicesInited = false;
std::vector<Serial::Device>		Serial::sDevices;

struct Serial::Impl {
	Impl( const Serial::Device &device, int baudRate );
	~Impl();

#if defined( CINDER_MSW )
	::HANDLE		mDeviceHandle;
	::COMMTIMEOUTS 	mSavedTimeouts;
#elif defined( CINDER_POSIX )
	int				mFd;
	::termios		mSavedOptions;
#endif
};

Serial::Serial( const Serial::Device &device, int baudRate )
	: mDevice( device ), mImpl( new Impl( device, baudRate ) )
{
}

Serial::~Serial()
{
}

Serial::Impl::Impl( const Serial::Device &device, int baudRate )
{
#if defined( CINDER_POSIX )
	mFd = open( ( "/dev/" + device.getName() ).c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
	if( mFd == -1 ) {
		throw SerialExcOpenFailed();
	}
	
	termios options;
	tcgetattr( mFd, &mSavedOptions );
	options = mSavedOptions;

	int rateConstant = [&] {
		switch (baudRate) {
			case 300: return B300;
			case 1200: return B1200;
			case 2400: return B2400;
			case 4800: return B4800;
			case 9600: return B9600;
			case 19200: return B19200;
#if defined( B28800 )
			case 28800: return B28800;
#endif
			case 38400: return B38400;
			case 57600: return B57600;
			case 115200: return B115200;
			case 230400: return B230400;
			default: return B9600;
		}
	}();
	
	::cfsetispeed( &options, rateConstant );
	::cfsetospeed( &options, rateConstant );
	
	options.c_cflag |= (CLOCAL | CREAD);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	::tcsetattr( mFd, TCSANOW, &options );
#elif defined( CINDER_MSW )
	mDeviceHandle = ::CreateFileA( device.getPath().c_str(), GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0 );
	if( mDeviceHandle == INVALID_HANDLE_VALUE ) {
		throw SerialExcOpenFailed();
	}
	
	::COMMCONFIG config;
	::DWORD configSize = sizeof( ::COMMCONFIG );
	::GetCommConfig( mDeviceHandle, &config, &configSize );
	
	string settingsStr = "baud=" + toString( baudRate ) + " parity=N data=8 stop=1";
	if( ! ::BuildCommDCBA( settingsStr.c_str(), &config.dcb ) ) {
		throw SerialExcOpenFailed();	
	}	
	
	if( ! ::SetCommState( mDeviceHandle, &config.dcb ) ) {
		throw SerialExcOpenFailed();
	}
	
	::GetCommTimeouts( mDeviceHandle, &mSavedTimeouts );
	::COMMTIMEOUTS timeOuts( mSavedTimeouts );

	timeOuts.ReadIntervalTimeout = MAXDWORD;
	timeOuts.ReadTotalTimeoutMultiplier = 0;
	timeOuts.ReadTotalTimeoutConstant = 0;
	::SetCommTimeouts( mDeviceHandle, &timeOuts );
#endif
}

Serial::Impl::~Impl()
{
#if defined( CINDER_POSIX )
	// restore the termios from before we opened the port
	::tcsetattr( mFd, TCSANOW, &mSavedOptions );
	::close( mFd );
#elif defined( CINDER_MSW )
	::SetCommTimeouts( mDeviceHandle, &mSavedTimeouts );
	::CloseHandle( mDeviceHandle );
#endif
}

Serial::Device Serial::findDeviceByName( const std::string &name, bool forceRefresh )
{
	for( const auto& device : getDevices( forceRefresh ) ) {
		if ( device.getName() == name )
			return device;
	}

	return Serial::Device();
}

Serial::Device Serial::findDeviceByNameContains( const std::string &searchString, bool forceRefresh )
{
	for( const auto& device : getDevices( forceRefresh ) ) {
		if( device.getName().find( searchString ) != std::string::npos )
			return device;
	}

	return Serial::Device();
}
	
const std::vector<Serial::Device>& Serial::getDevices( bool forceRefresh )
{
	if( ( ! forceRefresh ) && ( sDevicesInited ) )
		return sDevices;

	sDevices.clear();

#if defined( CINDER_POSIX )	
	::DIR *dir;
	::dirent *entry;
	dir = ::opendir( "/dev" );

	if( ! dir ) {
		throw SerialExcDeviceEnumerationFailed();
	}
	else {
		while( ( entry = ::readdir( dir ) ) != NULL ) {
			std::string str( (char *)entry->d_name );
			if( ( str.substr( 0, 4 ) == "tty." ) || ( str.substr( 0, 3 ) == "cu." ) ) {
				sDevices.push_back( Serial::Device( str ) );
			}
		}
	}
	
	::closedir( dir );
#elif defined( CINDER_MSW )
	::HDEVINFO devInfoSet;
	::DWORD devCount = 0;
	::SP_DEVINFO_DATA devInfo;
	::SP_DEVICE_INTERFACE_DATA devInterface;
	DWORD size = 0;

	devInfoSet = ::SetupDiGetClassDevs( &GUID_SERENUM_BUS_ENUMERATOR, 0, 0, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE );
	if( devInfoSet == INVALID_HANDLE_VALUE )
		throw SerialExcDeviceEnumerationFailed();
	
	devInterface.cbSize = sizeof( ::SP_DEVICE_INTERFACE_DATA );
	while( ::SetupDiEnumDeviceInterfaces( devInfoSet, 0, &GUID_SERENUM_BUS_ENUMERATOR, devCount++, &devInterface ) ) {
		// See how large a buffer we require for the device interface details
		::SetupDiGetDeviceInterfaceDetail( devInfoSet, &devInterface, 0, 0, &size, 0 );
		devInfo.cbSize = sizeof( ::SP_DEVINFO_DATA );
		shared_ptr<::SP_DEVICE_INTERFACE_DETAIL_DATA> interfaceDetail( (::SP_DEVICE_INTERFACE_DETAIL_DATA*)calloc( 1, size ), free );
		if( interfaceDetail ) {
			interfaceDetail->cbSize = sizeof( ::SP_DEVICE_INTERFACE_DETAIL_DATA );
			devInfo.cbSize = sizeof( ::SP_DEVINFO_DATA );
			if( ! ::SetupDiGetDeviceInterfaceDetail( devInfoSet, &devInterface, interfaceDetail.get(), size, 0, &devInfo ) ) {
				continue;
			}

			char friendlyName[2048];
			size = sizeof( friendlyName );
			friendlyName[0] = 0;
			::DWORD propertyDataType;
			if( ! ::SetupDiGetDeviceRegistryPropertyA( devInfoSet, &devInfo, SPDRP_FRIENDLYNAME, &propertyDataType, (LPBYTE)friendlyName, size, 0 ) ) {
				continue;
			}

			sDevices.push_back( Serial::Device( string( friendlyName ), toUtf8( (char16_t*)interfaceDetail->DevicePath ) ) );
		}
	}
	
	::SetupDiDestroyDeviceInfoList(devInfoSet);

#endif

	sDevicesInited = true;
	return sDevices;
}

const Serial::Device& Serial::getDevice() const
{
	return mDevice;
}

void Serial::writeBytes( const void *data, size_t numBytes )
{
	size_t totalBytesWritten = 0;
	
	while( totalBytesWritten < numBytes ) {
#if defined( CINDER_POSIX )
		long bytesWritten = ::write( mImpl->mFd, data, numBytes - totalBytesWritten );
		if( ( bytesWritten == -1 ) && ( errno != EAGAIN ) )
			throw SerialExcWriteFailure();
#elif defined( CINDER_MSW )
		::DWORD bytesWritten;
		if( ! ::WriteFile( mImpl->mDeviceHandle, data, static_cast<DWORD>( numBytes - totalBytesWritten ), &bytesWritten, 0 ) )
			throw SerialExcWriteFailure();
#endif
		if( bytesWritten != -1 )
			totalBytesWritten += bytesWritten;
	}
}

void Serial::readBytes( void *data, size_t numBytes )
{
	size_t totalBytesRead = 0;
	while( totalBytesRead < numBytes ) {
#if defined( CINDER_POSIX )
		long bytesRead = ::read( mImpl->mFd, data, numBytes - totalBytesRead );
		if( ( bytesRead == -1 ) && ( errno != EAGAIN ) )
			throw SerialExcReadFailure();
#elif defined( CINDER_MSW )
		::DWORD bytesRead = 0;
		if( ! ::ReadFile( mImpl->mDeviceHandle, data, static_cast<DWORD>( numBytes - totalBytesRead ), &bytesRead, 0 ) )
			throw SerialExcReadFailure();
#endif
		if( bytesRead != -1 )
			totalBytesRead += bytesRead;
		
		// yield thread time to the system
		this_thread::yield();
	}
}

size_t Serial::readAvailableBytes( void *data, size_t maximumBytes )
{
#if defined( CINDER_POSIX )
	long bytesRead = ::read( mImpl->mFd, data, maximumBytes );
#elif defined( CINDER_MSW )
	::DWORD bytesRead = 0;
	if( ! ::ReadFile( mImpl->mDeviceHandle, data, static_cast<DWORD>( maximumBytes ), &bytesRead, 0 ) )
		throw SerialExcReadFailure();
#endif

	if( bytesRead < 0 )
		bytesRead = 0;
		
	return (size_t)bytesRead;
}

void Serial::writeByte( uint8_t data )
{
	writeBytes( &data, 1 );
}

uint8_t Serial::readByte()
{
	uint8_t result;
	readBytes( &result, 1 );
	return result;
}

std::string Serial::readStringUntil( char token, size_t maxLength, double timeoutSeconds )
{
	std::string buffer;
	buffer.reserve( 1024 );

	bool useMaxLength = maxLength > 0;
	bool useTimer = timeoutSeconds > 0;
	Timer timer( useTimer );

	bool done = false;
	while( ! done ) {
		char v = readChar();
		buffer.push_back( v );
		if( v == token ) {
			done = true;
		}
		else if( useMaxLength && ( buffer.size() >= maxLength ) ) {
			done = true;
		}
		else if( useTimer && ( timer.getSeconds() > timeoutSeconds ) ) {
			throw SerialTimeoutExc();
		}
	}
	return buffer;
}

void Serial::writeString( const std::string &str )
{
	writeBytes( str.data(), str.size() );
}

size_t Serial::getNumBytesAvailable() const
{
	int result;
	
#if defined( CINDER_POSIX )
	::ioctl( mImpl->mFd, FIONREAD, &result );
#elif defined( CINDER_MSW )
	::COMSTAT status;
	::DWORD error;
	if( ! ::ClearCommError( mImpl->mDeviceHandle, &error, &status ) )
		throw SerialExc( "Serial failure upon attempt to retrieve information on device handle" );
	else
		result = status.cbInQue;
#endif
	
	return result;
}
	
void Serial::flush( bool input, bool output )
{
#if defined( CINDER_POSIX )
	int queue;
	if( input && output )
		queue = TCIOFLUSH;
	else if( input )
		queue = TCIFLUSH;
	else if( output )
		queue = TCOFLUSH;
	else
		return;
	
	::tcflush( mImpl->mFd, queue );
#elif defined( CINDER_MSW )
	::DWORD flags = 0;
	flags |= ( input ) ? PURGE_RXCLEAR : 0;
	flags |= ( output ) ? PURGE_TXCLEAR : 0;
	
	if( input || output )
		::PurgeComm( mImpl->mDeviceHandle, flags );
#endif
}

} // namespace cinder
