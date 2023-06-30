#pragma once

//extern "C"{
#include <arv.h>
//}

#include <mutex>
#include <atomic>

#include "ofMain.h"
#include "ofxOpenCv.h"

//template<typename Type>
//class Config{

//	Config():bSet(false){}

//	void set(const Type& val){
//		value = val;
//		bSet = true;
//	}

//	const Type& getValue(){
//		return value;
//	}

//	bool isSet(){
//		return bSet;
//	}

//private:
//	bool bSet;
//	Type value;
//};

#include <chrono>

namespace ofxAravis {
    struct Device {
        std::string id;
        std::string physical_id;
        std::string address;
        std::string vendor;
        std::string manufacturer_info;
        std::string model;
        std::string serial_nbr;
        std::string protocol;
    };

    class Grabber{
        public:
            using Clock = std::chrono::high_resolution_clock;

            Grabber();
            ~Grabber();

            void setPixelFormat(ArvPixelFormat format);
            bool setup(int targetX = -1, int targetY = -1, int targetWidth = -1, int targetHeight = -1, const char * targetPixelFormat = "");
            bool isInitialized();
            void stop();
        
            double getTemperature();
        
            std::vector<Device> & listDevices();
            std::vector<std::string> & listFormats();
        
            void setExposure(double exposure);
            int getSensorWidth();
            int getSensorHeight();

            void update();

            void draw(int x=0, int y=0, int w=0, int h=0);
            Clock::time_point last_frame();

            ArvCamera* camera;
            ArvStream* stream;
        
            void setExposureValue( double value );
            void setExposureAuto( ArvAuto value ); // ARV_AUTO_OFF or ARV_AUTO_ONCE or ARV_AUTO_CONTINUOUS
        
            double getExposureValue();
            ArvAuto getExposureAuto(); // ARV_AUTO_OFF or ARV_AUTO_ONCE or ARV_AUTO_CONTINUOUS

        private:
            static void onNewBuffer(ArvStream * stream, Grabber * aravis);
            void setPixels(cv::Mat& mat);
            
            vector<Device> devices;
            vector<std::string> formats;
        
            int araX, araY, araWidth, araHeight; const char * araPixelFormat; // what gets sent to API
            int initX, initY, initWidth, initHeight, initExp, initExpAuto; const char * initPixelFormat; // default dimensions reported from API
            int x, y, width, height; const char * pixelFormat; // dimensions reported back from API
            int w, h; // drawing width and height
        
            int sensorWidth, sensorHeight;
        
            ArvPixelFormat targetPixelFormat = ARV_PIXEL_FORMAT_BAYER_RG_8;
            std::mutex mutex;
            std::atomic_bool bFrameNew;
            ofImage image;
            cv::Mat mat;
            ofImageType imageType;
            ArvBuffer *buffer;
            Clock::time_point p_last_frame;
    };

}
