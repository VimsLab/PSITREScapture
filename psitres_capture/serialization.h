#include "stdafx.h"
#pragma once

#include <FlyCapture2.h>
#include <boost\filesystem.hpp>

template<typename T>
void writeMetadata(const boost::filesystem::path& p, const std::string& name, const T& t){
	std::ofstream ofs(p.string().c_str());
	assert_throw(ofs.good());
	boost::archive::xml_oarchive ar(ofs);
	ar << boost::serialization::make_nvp(name.c_str(), t);
}

namespace boost {
	namespace serialization {
		template<class Archive>
		void serialize(Archive& ar, FlyCapture2::ImageMetadata& md, const unsigned int version)
		{
			ar & make_nvp("embeddedTimeStamp", md.embeddedTimeStamp);
			ar & make_nvp("embeddedGain", md.embeddedGain);
			ar & make_nvp("embeddedShutter", md.embeddedShutter);
			ar & make_nvp("embeddedBrightness", md.embeddedBrightness);
			ar & make_nvp("embeddedExposure", md.embeddedExposure);
			ar & make_nvp("embeddedWhiteBalance", md.embeddedWhiteBalance);
			ar & make_nvp("embeddedFrameCounter", md.embeddedFrameCounter);
			ar & make_nvp("embeddedStrobePattern", md.embeddedStrobePattern);
			ar & make_nvp("embeddedGPIOPinState", md.embeddedGPIOPinState);
			ar & make_nvp("embeddedROIPosition", md.embeddedROIPosition);
			//ar & make_nvp("reserved", md.reserved);
		}
		template<class Archive>
		void serialize(Archive& ar, FlyCapture2::CameraInfo& camInfo, const unsigned int version){
			ar & make_nvp("applicationIPAddress", camInfo.applicationIPAddress);
			ar & make_nvp("applicationPort", camInfo.applicationPort);
			ar & make_nvp("bayerTileFormat", camInfo.bayerTileFormat);
			ar & make_nvp("busNumber", camInfo.busNumber);
			ar & make_nvp("ccpStatus", camInfo.ccpStatus);
			//ar & make_nvp("configROM", camInfo.configROM);
			//ar & make_nvp("defaultGateway", camInfo.defaultGateway);
			ar & make_nvp("driverName", string(camInfo.driverName));
			ar & make_nvp("driverType", camInfo.driverType);
			ar & make_nvp("firmwareBuildTime", string(camInfo.firmwareBuildTime));
			ar & make_nvp("firmwareVersion", string(camInfo.firmwareVersion));
			ar & make_nvp("gigEMajorVersion", camInfo.gigEMajorVersion);
			ar & make_nvp("gigEMinorVersion", camInfo.gigEMinorVersion);
			ar & make_nvp("iidcVer", camInfo.iidcVer);
			ar & make_nvp("interfaceType", camInfo.interfaceType);
			//ar & make_nvp("ipAddress", camInfo.ipAddress);
			ar & make_nvp("isColorCamera", camInfo.isColorCamera);
			//ar & make_nvp("macAddress", camInfo.macAddress);
			ar & make_nvp("maximumBusSpeed", camInfo.maximumBusSpeed);
			ar & make_nvp("modelName", string(camInfo.modelName));
			ar & make_nvp("nodeNumber", camInfo.nodeNumber);
			ar & make_nvp("pcieBusSpeed", camInfo.pcieBusSpeed);
			//ar & make_nvp("reserved", camInfo.reserved);
			ar & make_nvp("sensorInfo", string(camInfo.sensorInfo));
			ar & make_nvp("sensorResolution", string(camInfo.sensorResolution));
			ar & make_nvp("serialNumber", camInfo.serialNumber);
			//ar & make_nvp("subnetMask", camInfo.subnetMask);
			ar & make_nvp("userDefinedName", string(camInfo.userDefinedName));
			ar & make_nvp("vendorName", string(camInfo.vendorName));
			ar & make_nvp("xmlURL1", string(camInfo.xmlURL1));
			ar & make_nvp("xmlURL2", string(camInfo.xmlURL2));
		}
		template<class Archive>
		void serialize(Archive& ar, FlyCapture2::SystemInfo& sysInfo, const unsigned int version){
			ar & make_nvp("byteOrder", sysInfo.byteOrder);
			ar & make_nvp("cpuDescription", string(sysInfo.cpuDescription));
			ar & make_nvp("driverList", string(sysInfo.driverList));
			ar & make_nvp("gpuDescription", string(sysInfo.gpuDescription));
			ar & make_nvp("libraryList", string(sysInfo.libraryList));
			ar & make_nvp("numCpuCores", sysInfo.numCpuCores);
			ar & make_nvp("osDescription", string(sysInfo.osDescription));
			ar & make_nvp("osType", sysInfo.osType);
			//ar & make_nvp("reserved", sysInfo.reserved);
			ar & make_nvp("screenHeight", sysInfo.screenHeight);
			ar & make_nvp("screenWidth", sysInfo.screenWidth);
			ar & make_nvp("sysMemSize", sysInfo.sysMemSize);
		}
		template<class Archive>
		void serialize(Archive& ar, FlyCapture2::FC2Version& fc2Info, const unsigned int version){
			ar & make_nvp("build", fc2Info.build);
			ar & make_nvp("major", fc2Info.major);
			ar & make_nvp("minor", fc2Info.minor);
			ar & make_nvp("type", fc2Info.type);
		}
	}
}