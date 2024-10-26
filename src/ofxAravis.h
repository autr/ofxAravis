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

    struct Feature {
        std::string key;
        std::string value;
    };
    struct Bounds {
        double min;
        double max;
    };

    struct TriggerSource {
        std::string todo;
    };

    Device GetDeviceInfo( int idx );
    std::vector<Device> ListAllDevices( bool print = true );
    void HandleError( GError * err );
    std::vector<std::string> ArrayToVector( const char ** array, int length );

    class Grabber{
        public:
            using Clock = std::chrono::high_resolution_clock;
            using BufferCallback = std::function<void(const cv::Mat&)>;
            void setBufferCallback(BufferCallback callback);
            BufferCallback bufferCallback; // Store the callback function

            Grabber();
            ~Grabber();

            void onAppExit(ofEventArgs& args);
            void setPixelFormat(ArvPixelFormat format);
            bool setup(int targetCamera = 0, int targetX = -1, int targetY = -1, int targetWidth = -1, int targetHeight = -1, const char * targetPixelFormat = "");
            bool isInitialized();
            void stop();
        
            double getTemperature();
            
            std::vector<std::string> & listFormats();
            Device & getInfo();
        
            void setExposure(double exposure);
            int getSensorWidth();
            int getSensorHeight();

            bool update();

            void draw(int x=0, int y=0, int w=0, int h=0);
            void drawInfo( int x = 10, int y = 20 );
            Clock::time_point last_frame();

            ArvCamera* camera;
            ArvStream* stream;
            std::vector<std::string> availableTriggerModes;
            std::vector<std::string> availableTriggerSources;
        
            ofTexture & getTexture();
            int totalFrames;
        
            bool isInited();
        
            // ------- EXPOSURE TIME -------
        
            bool hasExposureTime();
            void setExposureTime( double value );
            double getExposureTime();
            Bounds getExposureBounds();
        
            // ------- EXPOSURE TIME AUTO -------
        
            bool hasExposureTimeAuto();
            void setExposureTimeAuto( ArvAuto mode );
            ArvAuto getExposureTimeAuto();
        
        
            // ------- TRIGGER MODE -------
        
            void setTriggerMode( std::string key );
            std::vector<std::string> getAvailableTriggerModes();
        //arv_camera_dup_available_triggers
        
            // ------- TRIGGER SOURCE -------
        
            void setTriggerSource( std::string key );
            std::string getTriggerSource();
            std::vector<std::string> getAvailableTriggerSources();
        
            // ------- ENUMERATIONS -------
            
            vector<std::string> getAvailableEnumerations( std::string key );
        
            // ------- FEATURES -------
        
            ofJson listAllFeatures();
        
            void setFeatureString( std::string key, std::string value );
            std::string getFeatureString( std::string key );
        
            void setFeatureBoolean( std::string key, bool value );
            bool getFeatureBoolean( std::string key );
        
            void setFeatureInteger( std::string key, int value );
            int getFeatureInteger( std::string key );
            
            void setFeatureFloat( std::string key, int value );
            float getFeatureFloat( std::string key );
        
            void executeCommand( std::string command );
        
            // ------- FPS -------

            void setFPS( double fps );
            double getFPS();
            double getMinFPS();
            double getMaxFPS();
            float getActualFPS();
            float previousTimestamp;
            float fpsTimeElapsed;
        
            // ------- FORMATS -------
        
            void setPixelFormat( std::string format );
            std::string getPixelFormat();
            bool formatIsFound = false;
        
            // ------- XML -------
        
            std::string getGenicamXML();
        
            int getWidth();
            int getHeight();
            

        private:
            static void onNewBuffer(ArvStream * stream, Grabber * aravis);
            void setPixels(cv::Mat& mat);

            std::string safeConvertChars( const char * chars );

            void loopThroughFeatures( ofJson & features, ArvGc * genicam, const char * feature, int level );
            void processFeatureNode( ofJson & entry, ArvGcFeatureNode * node, int level );
            
            Device info;
            vector<std::string> formats;
        
            int araX, araY, araWidth, araHeight; const char * araPixelFormat; // what gets sent to API
            int initX, initY, initWidth, initHeight, initExp, initExpAuto; const char * initPixelFormat; // default dimensions reported from API
            int x, y, width, height; const char * pixelFormat; // dimensions reported back from API
            int w, h; // drawing width and height
            int sensorWidth, sensorHeight;
            bool inited = false;
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
