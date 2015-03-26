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
#include <boost\program_options.hpp>
#include <sstream>
#include <cstdint>
#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <algorithm>

using namespace FlyCapture2;
using namespace std;
using namespace cv;
using namespace tbb::flow;
using namespace tbb;
using namespace boost::filesystem;
using namespace boost::posix_time;
using namespace boost::serialization;
using namespace boost::archive;
using namespace boost::program_options;

struct SourceNode;
struct PGCam;
struct FlowData;
struct SharedResources;
struct GUI;
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

map<uint32_t, SharedResources> resources;

struct SharedResources{
	SharedResources() :frameno(){
		frameno = 0;
	}
	atomic<int> frameno;
};

template<typename T>
void writeMetadata(const path& p, const string& name, const T& t){
	ofstream ofs(p.string().c_str());
	assert_throw(ofs.good());
	xml_oarchive ar(ofs);
	ar << make_nvp(name.c_str(), t);
}

struct PGCam{
	PGCam(const uint32_t& serial, const path& BASE_PATH, const locale& TS_LOCALE, const bool& start_capture) :serial(serial), BASE_PATH(BASE_PATH), TS_LOCALE(TS_LOCALE), timestamp(microsec_clock::local_time()){
		SystemInfo sysInfo;
		PG_Call(Utilities::GetSystemInfo(&sysInfo));
		writeMetadata(getMetadataPath("SystemInfo"), "SystemInfo", sysInfo);

		FC2Version fc2Info;
		PG_Call(Utilities::GetLibraryVersion(&fc2Info));
		writeMetadata(getMetadataPath("FC2Version"), "FC2Version", fc2Info);

		PGRGuid guid;
		BusManager bm;
		PG_Call(bm.GetCameraFromSerialNumber(serial, &guid));
		PG_Call(cam.Connect(&guid));
		assert_throw(cam.IsConnected());
		try{
			CameraInfo camInfo;
			PG_Call(cam.GetCameraInfo(&camInfo));
			writeMetadata(getMetadataPath("CameraInfo"), "CameraInfo", camInfo);

			if (start_capture)
				PG_Call(cam.StartCapture(OnImageGrabbed, this));
		}
		catch (...){
			PG_Call(cam.Disconnect());
			throw;
		}
	}
	~PGCam(){
		try{
			PG_Call(cam.StopCapture());
		}
		catch (...){
			cerr << "..." << endl;
		}
		try{
			PG_Call(cam.Disconnect());
		}
		catch (...){
			cerr << "..." << endl;
		}
	}
	path getMetadataPath(const string& structName){
		ostringstream tsStr;
		tsStr.imbue(TS_LOCALE);
		tsStr << timestamp;
		path p(BASE_PATH);
		p /= tsStr.str();
		p += "_";
		p += to_string(serial);
		p += "_";
		p += structName;
		p += ".xml";
		return p;
	}
	const uint32_t serial;
	const path BASE_PATH;
	const locale TS_LOCALE;
	const locale HOUR_LOCALE = locale(cout.getloc(), new time_facet("%H"));
	const ptime timestamp;
	Camera cam;
};

void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	try{
		ptime timestamp = microsec_clock::local_time();
		const PGCam* cam = static_cast<const PGCam*>(pCallbackData);
		int frameno = ++resources[cam->serial].frameno;
		
		path datePath;
		datePath = cam->BASE_PATH;
		datePath /= to_iso_string(timestamp.date());
		create_directories(datePath);

		ostringstream hourStr;
		hourStr.imbue(cam->HOUR_LOCALE);
		hourStr << timestamp;
		
		path hourPath;
		hourPath = datePath;
		hourPath /= hourStr.str();
		create_directories(hourPath);

		ostringstream tsStr;
		tsStr.imbue(cam->TS_LOCALE);
		tsStr << timestamp;

		path imgPath;
		imgPath = hourPath;
		imgPath /= tsStr.str();
		imgPath += "_";
		imgPath += to_string(cam->serial);
		imgPath += "_";
		imgPath += to_string(frameno);
		imgPath += ".jpg";
		
		path metaPath(imgPath);
		metaPath += "_ImageMetadata.xml";
		
		pImage->Save(imgPath.string().c_str());
		writeMetadata(metaPath, "ImageMetadata", pImage->GetMetadata());

		Image bgrImage;
		PG_Call(pImage->Convert(PIXEL_FORMAT_BGR, &bgrImage));
		Mat bgrMat = Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData());
		Mat display = Mat::zeros(512, 612, CV_8UC3);
		resize(bgrMat, display, display.size());
		imshow(to_string(cam->serial), display);

		ostringstream infoStr;
		infoStr << timestamp << " " << cam->serial << " " << frameno << endl;
		cerr << infoStr.str();
	}
	catch (const exception& e){
		cerr << e.what() << endl;
	}
	catch (...){
		cerr << "..." << endl;
	}
}

enum KeyedAction{
	CONTINUE = 0,
	QUIT = 1
};

struct GUI{
	GUI(const vector<Ptr<PGCam> >& cams, const float& fps) :cams(cams), fps(fps > 0 ? fps : 60){
		actionMap["quit"] = KeyedAction::QUIT;
	}
	void mainLoop(){
		uint32_t flags = KeyedAction::CONTINUE;
		do{
			try{
				/*for (vector<Ptr<PGCam> >::const_iterator pCamIt = cams.cbegin(); pCamIt != cams.cend(); pCamIt++){
					namedWindow(to_string((*pCamIt)->serial));
					}*/
				flags |= waitKeyEvent();
			}
			catch (const exception& e){
				cerr << e.what() << endl;
			}
			catch (...){
				cerr << "..." << endl;
			}
		} while (!(flags & KeyedAction::QUIT));
	}
	KeyedAction waitKeyEvent(){		
		int resp = waitKey(int(1000 / fps));
		if (resp >= 0){
			char key = resp;
			history += tolower(key);
			times.push_back(microsec_clock::local_time());

			int i = 0;
			while ((i < times.size()) && (time_period(times[i], times.back()).length() > seconds(2)))
				i++;

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
	const vector<Ptr<PGCam> > cams;
	const float fps;
	map<string, KeyedAction> actionMap;
	string history;
	vector<ptime> times;
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

int psitres_capture(int argc, _TCHAR* argv[]){
	const string config_file = "config.ini";

	options_description config("psitres_config");
	config.add_options()
		("output_directory", value<string>(), "directory where captured images are stored")
		("display_fps", value<float>(), "maximum FPS when displaying captured images")
		("sync_capture", value<bool>(), "starts cameras using Camera::StartSyncCapture for testing with 1394 cameras")
		("pg_serial", value<vector<uint32_t> >(), "point grey camera serial number(s) to be initialized");

	variables_map vm;
	ifstream ifs(config_file.c_str());
	assert_throw(ifs);
	store(parse_config_file(ifs, config), vm);
	notify(vm);

	static const path BASE_PATH(vm["output_directory"].as<string>());
	static const vector<uint32_t> PG_SERIALS = vm.find("pg_serial") != vm.end() ? vm["pg_serial"].as< vector<uint32_t> >() : vector<uint32_t>();
	static const float FPS = vm["display_fps"].as<float>();
	static const locale TS_LOCALE = locale(cout.getloc(), new time_facet("%Y%m%dT%H%M%S%F"));
	create_directories(BASE_PATH);

	vector<Ptr<PGCam> > cams;
	try{
		for (vector<uint32_t>::const_iterator serialIt = PG_SERIALS.cbegin(); serialIt != PG_SERIALS.cend(); serialIt++){
			namedWindow(to_string(*serialIt));
			cams.push_back(new PGCam(*serialIt, BASE_PATH, TS_LOCALE, !vm["sync_capture"].as<bool>()));
		}

		if (vm["sync_capture"].as<bool>()){
			vector<const Camera*> ppCameras;
			vector<const ImageEventCallback> pCallbackFns;
			vector<const PGCam*> pCallbackDataArray;
			for (int i = 0; i < cams.size(); i++){
				ppCameras.push_back(&cams[i]->cam);
				pCallbackFns.push_back(OnImageGrabbed);
				pCallbackDataArray.push_back(cams[i]);
			}
			PG_Call(Camera::StartSyncCapture(cams.size(), ppCameras.data(), pCallbackFns.data(), (const void**)pCallbackDataArray.data()));
		}

		if (cams.size() > 0){
			GUI gui(cams, FPS);
			gui.mainLoop();
		}
	}
	catch (...){
		cams.clear();
		destroyAllWindows();
		throw;
	}

	cams.clear();
	destroyAllWindows();
	return EXIT_SUCCESS;
}

int _tmain(int argc, _TCHAR* argv[])
{
	int ret = EXIT_FAILURE;
	try {
		ret = psitres_capture(argc, argv);
	}
	catch (const exception& e) {
		cerr << e.what() << endl;
	}
	catch (...) {
		cerr << "..." << endl;
	}
	getchar();
	return ret;
}

