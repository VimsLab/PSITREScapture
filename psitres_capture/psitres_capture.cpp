#include "stdafx.h"
#include "errorutils.h"
#include <Windows.h>
#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <tbb\flow_graph.h>
#include <tbb\atomic.h>
#include <boost\filesystem.hpp>
#include <boost\date_time\posix_time\posix_time.hpp>
#include <boost\archive\xml_oarchive.hpp>
#include <boost\serialization\nvp.hpp>
#include <sstream>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <string>
#include <sstream>

using namespace FlyCapture2;
using namespace std;
using namespace cv;
using namespace tbb::flow;
using namespace tbb;
using namespace boost::filesystem;
using namespace boost::posix_time;
using namespace boost::serialization;
using namespace boost::archive;

struct GUI;
struct FlowData;
struct PGCam;
struct SourceNode;
struct SharedResources;
enum KeyedAction;
void OnImageGrabbed(Image*, const void*);
template<typename T>
void writeMetadata(const path& p, const string& name, const T& t);
namespace boost {
	namespace serialization {
		template<class Archive>
		void serialize(Archive&, ImageMetadata&, const unsigned int);
		template<class Archive>
		void serialize(Archive&, CameraInfo&, const unsigned int);
	}
}

// TODO: name this better
struct SharedResources{
	SharedResources() :display(Mat::zeros(512, 612, CV_8UC3)), frameno(){
		frameno = 0;
	}
	Mat display;
	atomic<int> frameno;
};

struct FlowData{
	FlowData(const uint32_t& serial, const ptime& timestamp, const Image& image) :serial(serial), timestamp(timestamp), image(image){}
	FlowData() :serial(), timestamp(), image(){}
	uint32_t serial;
	ptime timestamp;
	Image image;
};

template<typename T>
void writeMetadata(const path& p, const string& name, const T& t){
	ofstream ofs(p.string().c_str());
	assert_throw(ofs.good());
	xml_oarchive ar(ofs);
	ar << make_nvp(name.c_str(), t);
}

struct PGCam{
	PGCam(const uint32_t& serial, const path& BASE_PATH, const locale& TS_LOCALE, function_node<FlowData>& source_node, SharedResources& resource) :serial(serial), BASE_PATH(BASE_PATH), TS_LOCALE(TS_LOCALE), source_node(source_node), resource(resource), timestamp(microsec_clock::local_time()){
		SystemInfo sysInfo;
		PG_Call(Utilities::GetSystemInfo(&sysInfo));
		writeMetadata(getMetadataPath("SystemInfo"), "SystemInfo", sysInfo);

		FC2Version fc2Info;
		PG_Call(Utilities::GetLibraryVersion(&fc2Info));
		writeMetadata(getMetadataPath("FC2Version"), "FC2Version", fc2Info);

		PGRGuid guid;
		PG_Call(BusManager().GetCameraFromSerialNumber(serial, &guid));
		PG_Call(cam.Connect(&guid));
		assert_throw(cam.IsConnected());
		try{
			CameraInfo camInfo;
			PG_Call(cam.GetCameraInfo(&camInfo));
			writeMetadata(getMetadataPath("CameraInfo"), "CameraInfo", camInfo);

			PG_Call(cam.StartCapture(OnImageGrabbed, this));			
		}
		catch (const exception&){
			PG_Call(cam.Disconnect());
			throw;
		}
	}
	Mat read() const{
		return resource.display;
	}
	~PGCam(){
		PG_Call(cam.StopCapture());
		PG_Call(cam.Disconnect());
	}
	path getMetadataPath(const string& structName){
		ostringstream tsStr;
		tsStr.imbue(TS_LOCALE);
		tsStr << timestamp;
		path p(BASE_PATH);
		p /= tsStr.str();
		p += ".";
		p += to_string(serial);
		p += ".";
		p += structName;
		p += ".xml";
		return p;
	}
	const uint32_t serial;
	const path BASE_PATH;
	const locale TS_LOCALE;
	function_node<FlowData>& source_node;
	SharedResources& resource;
	GigECamera cam;
	ptime timestamp;
};

void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	ptime timestamp = microsec_clock::local_time();
	const PGCam* cam = static_cast<const PGCam*>(pCallbackData);
	Image image;
	image.DeepCopy(pImage);
	FlowData fdata(cam->serial, timestamp, image);
	cam->source_node.try_put(fdata);
}

enum KeyedAction{
	CONTINUE = 0,
	QUIT = 1
};

struct GUI{
	GUI(const vector<Ptr<const PGCam> >& cams, const float& fps) :cams(cams), fps(fps > 0 ? fps : 1){
		actionMap["quit"] = KeyedAction::QUIT;
	}
	void mainLoop(){
		//int frame_no = 0;
		uint32_t flags = KeyedAction::CONTINUE;
		do{
			for (vector<Ptr<const PGCam> >::const_iterator pCamIt = cams.cbegin(); pCamIt != cams.cend(); pCamIt++){
				Mat display = (*pCamIt)->read();
				imshow(to_string((*pCamIt)->serial), display);
			}
			flags |= waitKeyEvent();
		} while (!(flags & KeyedAction::QUIT));// && (++frame_no < (fps * 15)));
	}
	KeyedAction waitKeyEvent(){
		int resp = waitKey(int(1000 / fps));
		if (resp >= 0){
			char key = resp;
			history += tolower(key);
			times.push_back(microsec_clock::local_time());

			int i;
			for (i = 0; (i < times.size()) && (time_period(times[i], times.back()).length() > seconds(2)); i++)
				;
			history = string(history.begin() + i, history.end());
			times = vector<ptime>(times.begin() + i, times.end());
			
			map<string, KeyedAction>::iterator actIt;
			for (actIt = actionMap.begin(); actIt != actionMap.end(); actIt++){
				size_t pos = history.find(actIt->first);
				if (pos != string::npos){
					cout << ">>>" << actIt->first << "<<<" << endl;
					history.clear();
					times.clear();
					return actIt->second;
				}
			}
		}

		return KeyedAction::CONTINUE;
	}
	const vector<Ptr<const PGCam> > cams;
	const float fps;
	string history;
	vector<ptime> times;
	map<string, KeyedAction> actionMap;
};

namespace boost {
	namespace serialization {
		template<class Archive>
		void serialize(Archive& ar, ImageMetadata& md, const unsigned int version)
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
		void serialize(Archive& ar, CameraInfo& camInfo, const unsigned int version){
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
		void serialize(Archive& ar, SystemInfo& sysInfo, const unsigned int version){
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
		void serialize(Archive& ar, FC2Version& fc2Info, const unsigned int version){
			ar & make_nvp("build", fc2Info.build);
			ar & make_nvp("major", fc2Info.major);
			ar & make_nvp("minor", fc2Info.minor);
			ar & make_nvp("type", fc2Info.type);
		}
	}
}

struct SourceNode{
	SourceNode(const path& BASE_PATH, const locale& TS_LOCALE, map<uint32_t, SharedResources>& resources) : BASE_PATH(BASE_PATH), TS_LOCALE(TS_LOCALE), resources(resources){
	}
	void operator() (FlowData data) {
		try{
			Image bgrImage;
			PG_Call(data.image.Convert(PIXEL_FORMAT_BGR, &bgrImage));

			Mat display(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData());
			resize(display, resources[data.serial].display, resources[data.serial].display.size());

			resources[data.serial].frameno++;

			path datePath;
			datePath = BASE_PATH;
			datePath /= to_iso_string(data.timestamp.date());
			create_directories(datePath);

			ostringstream hourStr;
			path hourPath;
			hourPath = datePath;
			hourStr.imbue(HOUR_LOCALE);
			hourStr << data.timestamp;
			hourPath /= hourStr.str();
			create_directories(hourPath);

			ostringstream tsStr;
			tsStr.imbue(TS_LOCALE);
			tsStr << data.timestamp;

			path imgPath;
			imgPath = hourPath;
			imgPath /= tsStr.str();
			imgPath += ".";
			imgPath += to_string(data.serial);
			imgPath += ".jpg";
			bgrImage.Save(imgPath.string().c_str());

			path metaPath;
			metaPath = hourPath;
			metaPath /= tsStr.str();
			metaPath += ".";
			metaPath += to_string(data.serial);
			metaPath += ".ImageMetadata.xml";
			writeMetadata(metaPath, "ImageMetadata", data.image.GetMetadata());

			/*
			ostringstream infoStr;
			time_duration dur = time_period(data.timestamp, microsec_clock::local_time()).length();
			infoStr << data.serial << ": " << resources[data.serial].frameno << " -> " << dur << '\n';
			cerr << infoStr.str();
			*/
		}
		catch (const exception& e){
			cerr << e.what() << endl;
			throw;
		}
	}
	const path BASE_PATH;
	const locale TS_LOCALE;
	const locale HOUR_LOCALE = locale(cout.getloc(), new time_facet("%H"));
	map<uint32_t, SharedResources>& resources;
};

int psitres_capture(int argc, _TCHAR* argv[]){
	static const std::array<uint32_t, 2> PG_SERIALS = { 12010990, 12010988 };
#ifndef _DEBUG
	static const path BASE_PATH = "E:\\stereo\\";
#else
	static const path BASE_PATH = "E:\\stereod\\";
#endif
	static const locale TS_LOCALE = locale(cout.getloc(), new time_facet("%Y%m%dT%H%M%S%F"));
	static const float FPS = 30;

	graph g;
	map<uint32_t, SharedResources> resources;
	function_node<FlowData> sourceNode(g, unlimited, SourceNode(BASE_PATH, TS_LOCALE, resources));
	vector<Ptr<const PGCam> > cams;
	try{
		for (std::array<uint32_t, 2>::const_iterator serialIt = PG_SERIALS.cbegin(); serialIt < PG_SERIALS.cend(); serialIt++){
			cams.push_back(new PGCam(*serialIt, BASE_PATH, TS_LOCALE, sourceNode, resources[*serialIt]));
		}
		GUI gui(cams, FPS);
		gui.mainLoop();
	}
	catch (const exception&){
		cams.clear();
		g.wait_for_all();
		throw;
	}

	cams.clear();
	g.wait_for_all();
	return 0;
}

int _tmain(int argc, _TCHAR* argv[])
{
	try {
		return psitres_capture(argc, argv);
	}
	catch (const exception& e) {
		cerr << e.what() << endl;
	}
}

