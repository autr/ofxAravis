#pragma once


/* ====================================== */
/*                                        */
/*               OFXGENICAM               */
/*                                        */
/* ====================================== */

#include "ofMain.h"

#include <arv.h>
#include <mutex>
#include <atomic>
#include <chrono>

namespace ofxGenicam {

    // ====== GENERAL ======

    ofJson listDevices( bool print = true );

    class Camera {

        public:

            // ====== CALLBACKS ======

            using BufferCallback = std::function<void( void * rawPixels, int width, int height, int bitsPerPixel, std::string pixelFormat )>;
            using ErrorCallback = std::function<void(std::string origin, std::string message)>;

            void setBufferCallback( BufferCallback callback );
            void setErrorCallback( ErrorCallback callback );

            BufferCallback bufferCallback;
            ErrorCallback errorCallback;

            // ====== SETUP ======

            Camera();
            bool open( int index = 0 );
            ~Camera();

            void onAppExit(ofEventArgs& args);

            // ====== STREAM ======

            bool start( int numberOfBuffers = 2 ); // less = faster, more = less glitchy
            bool stop();
        
            // ====== FEATURES ======
        
            std::string getGenicamXML();
            ofJson listAllFeatures( bool print = true );
        
            bool setStr( std::string key, std::string value );
            std::string getStr( std::string key );
        
            bool setBool( std::string key, bool value );
            bool getBool( std::string key );
        
            bool setInt( std::string key, int value );
            int getInt( std::string key );
            
            bool setFloat( std::string key, float value );
            float getFloat( std::string key );
        
            bool executeCommand( std::string command );
            std::function<void(ArvStream*)> bufferCallbackWrapper;

        private:
        
        
            ArvCamera * camera;
            
            // ====== ERRORS ======
            
            bool handleError( GError * err, std::string origin);

            // ====== STREAM ======

            bool isStreaming = false;

            ArvStream * stream = nullptr;
            using Clock = std::chrono::high_resolution_clock;

            ArvBuffer * buffer = nullptr;
            std::mutex mutex;
            std::atomic_bool bFrameNew;

            Clock::time_point lastFrame();
            Clock::time_point lastFrameTime;

            std::string pixelFormat;

            static void onNewBuffer(ArvStream * stream, Camera * aravis);

            // ====== STATS ======

            ofJson stats;

            // ====== UTILITIES ======

            std::string safeConvertChars( const char * chars );
        

            // ====== FEATURES ======

            void loopThroughFeatures( ofJson & features, ArvGc * genicam, const char * feature, int level );
            void processFeatureNode( ofJson & entry, ArvGcFeatureNode * node, int level );

    };
    


}
