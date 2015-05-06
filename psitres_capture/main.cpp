#include "stdafx.h"

#include "errorutils.h"
#include "serialization.h"
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


// forward declarations
struct PGCam;
struct FlowData;
struct SharedResources;
struct GUI;
enum KeyedAction;
void OnImageGrabbed(Image*, const void*);


struct SharedResources{
	SharedResources() :frameno(){
		frameno = 0;
	}
	atomic<int> frameno;
};


struct PGCam{
	PGCam(const uint32_t& serial, const path& BASE_PATH, const locale& TS_LOCALE, const bool& start_capture) 
		:serial(serial), BASE_PATH(BASE_PATH), TS_LOCALE(TS_LOCALE), timestamp(microsec_clock::local_time()){
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


struct FlowData{
	FlowData(const path& BASE_PATH, const locale& HOUR_LOCALE, const locale& TS_LOCALE, const int& serial, const ptime& timestamp, const int& frameno, const Image* pImage) :
		BASE_PATH(BASE_PATH), HOUR_LOCALE(HOUR_LOCALE), TS_LOCALE(TS_LOCALE), serial(serial), timestamp(timestamp), frameno(frameno){
		image.DeepCopy(pImage);
	}
	FlowData(){}
	path BASE_PATH;
	locale HOUR_LOCALE;
	locale TS_LOCALE;
	int serial;
	ptime timestamp;
	int frameno;
	Image image;
};


map<uint32_t, SharedResources> resources;
graph g;
function_node<FlowData> source(g, unlimited, [](FlowData data){
	try{
		path datePath;
		datePath = data.BASE_PATH;
		datePath /= to_iso_string(data.timestamp.date());
		//create_directories(datePath);

		ostringstream hourStr;
		hourStr.imbue(data.HOUR_LOCALE);
		hourStr << data.timestamp;

		path hourPath;
		hourPath = datePath;
		hourPath /= hourStr.str();
		//create_directories(hourPath);

		ostringstream tsStr;
		tsStr.imbue(data.TS_LOCALE);
		tsStr << data.timestamp;

		path imgPath;
		imgPath = hourPath;
		imgPath /= tsStr.str();
		imgPath += "_";
		imgPath += to_string(data.serial);
		imgPath += "_";
		imgPath += to_string(data.frameno);
		imgPath += ".jpg";

		path metaPath(imgPath);
		metaPath += "_ImageMetadata.xml";

		create_directories(hourPath);
		data.image.Save(imgPath.string().c_str());
		writeMetadata(metaPath, "ImageMetadata", data.image.GetMetadata());

		Image bgrImage;
		PG_Call(data.image.Convert(PIXEL_FORMAT_BGR, &bgrImage));
		Mat bgrMat = Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData());
		Mat display = Mat::zeros(512, 612, CV_8UC3);
		resize(bgrMat, display, display.size());
		imshow(to_string(data.serial), display);

		ostringstream infoStr;
		infoStr << data.timestamp << " " << data.serial << " " << data.frameno << endl;
		cerr << infoStr.str();
	}
	catch (const exception& e){
		cerr << e.what() << endl;
	}
	catch (...){
		cerr << "..." << endl;
	}
});


void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	try{
		ptime timestamp = microsec_clock::local_time();
		const PGCam* cam = static_cast<const PGCam*>(pCallbackData);
		int frameno = ++resources[cam->serial].frameno;

		FlowData data(cam->BASE_PATH, cam->HOUR_LOCALE, cam->TS_LOCALE, cam->serial, timestamp, frameno, pImage);
		source.try_put(data);
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
				flags ^= waitKeyEvent();
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
			cerr << "Initializing frame counter for " << *serialIt << " to " << resources[*serialIt].frameno;
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
		g.wait_for_all();
		destroyAllWindows();
		throw;
	}

	cams.clear();
	g.wait_for_all();
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
	cout << "Enter anything to exit..." << endl;
	getchar();
	return ret;
}

