#include <ofxAravis.h>
#include <opencv2/opencv.hpp>

namespace ofxAravis {


    ofTexture & Grabber::getTexture() {
        return image.getTexture();
    }

    void Grabber::onNewBuffer(ArvStream *stream, Grabber *aravis) {
        ArvBuffer *buffer;
        
        buffer = arv_stream_try_pop_buffer(stream);
        if (buffer != nullptr) {
            if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
                auto w = arv_buffer_get_image_width(buffer);
                auto h = arv_buffer_get_image_height(buffer);
                auto format = arv_buffer_get_image_pixel_format(buffer);
                
                cv::Mat matRgb(h, w, CV_8UC3);
                
                switch (format) {
                    case ARV_PIXEL_FORMAT_BAYER_RG_8: {
                        cv::Mat matBayer(h, w, CV_8UC1, const_cast<void *>(arv_buffer_get_data(buffer, nullptr)));
                        cv::cvtColor(matBayer, matRgb, CV_BayerRG2BGR);
                    }
                        break;
                    default:
                        ofLogError("ofxARavis") << "Unknown pixel format";
                }
                aravis->setPixels(matRgb);
            }
            arv_stream_push_buffer(stream, buffer);
        }
    }

    void Grabber::setPixels(cv::Mat &m) {
        mutex.lock();
        p_last_frame = Clock::now();
        mat = m.clone();
        mutex.unlock();
        bFrameNew = true;
    }

    ////////////////////////////////////////

    void Grabber::setExposureValue( double value ) {
        GError *err = nullptr;
        setExposureAuto( ARV_AUTO_OFF );
        arv_camera_set_exposure_time(camera, value, &err);
    }

    void Grabber::setExposureAuto( ArvAuto value ) {
        GError *err = nullptr;
        arv_camera_set_exposure_time_auto(camera, value, &err);
    }
    double Grabber::getExposureValue() {
        GError *err = nullptr;
        return arv_camera_get_exposure_time(camera, &err);
    }

    ArvAuto Grabber::getExposureAuto() {
        GError *err = nullptr;
        return arv_camera_get_exposure_time_auto(camera, &err);
    }

    void Grabber::setFPS( double fps ) {
        GError *err = nullptr;
        arv_camera_set_frame_rate(camera, fps, &err);
    }
    double Grabber::getFPS() {
        GError *err = nullptr;
        return arv_camera_get_frame_rate(camera, &err);
    }
    double Grabber::getMinFPS() {
        GError *err = nullptr;
        double min;
        double max;
        arv_camera_get_frame_rate_bounds( camera, &min, &max, &err);
        return min;
    }
    double Grabber::getMaxFPS() {
        GError *err = nullptr;
        double min;
        double max;
        arv_camera_get_frame_rate_bounds( camera, &min, &max, &err);
        return max;
    }

    void Grabber::update() {
        if (bFrameNew) {
            bFrameNew = false;
            mutex.lock();
            image.setFromPixels(mat.data, width, height, ofImageType::OF_IMAGE_COLOR);
            mutex.unlock();
        }
    }

    Device GetDeviceInfo( int idx ) {
        
        ofxAravis::Device device;
        
        device.id = arv_get_device_id(idx);
        device.physical_id = arv_get_device_physical_id(idx);
        device.address = arv_get_device_address(idx);
        device.vendor = arv_get_device_vendor(idx);
        device.manufacturer_info = arv_get_device_manufacturer_info(idx);
        device.model = arv_get_device_model(idx);
        device.serial_nbr = arv_get_device_serial_nbr(idx);
        device.protocol = arv_get_device_protocol(idx);
        
        return device;
    }

    std::vector<Device> ListAllDevices( bool print ){
        arv_update_device_list();
        std::vector<Device> devices;
        for (int i = 0; i < arv_get_n_devices(); i++) {
            
            ofxAravis::Device device = GetDeviceInfo( i );
            
            if (print) {
                
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "id: " + device.id;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "physical_id: " + device.physical_id;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "address: " + device.address;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "vendor: " + device.vendor;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "manufacturer_info :" + device.manufacturer_info;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "model: " + device.model;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "serial_nbr: " + device.serial_nbr;
                ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "protocol: " + device.protocol;
            }

            devices.push_back(device);
        }
        return devices;
    }

    Grabber::Grabber() {
        p_last_frame = Clock::now();
    }

    Grabber::~Grabber() {
        stop();
    }

    void Grabber::drawInfo( int x, int y ) {
        
        glm::vec2 center = glm::vec2( ofGetWidth()/2, ofGetHeight()/2 );
        
        std::string model = getInfo().model;
        std::string FPS = ofToString(getFPS());
        std::string minFPS = ofToString(getMinFPS());
        std::string maxFPS = ofToString(getMaxFPS());
        std::string appFPS = ofToString(int(ofGetFrameRate()));
        std::string temp = ofToString(getTemperature());
        std::string sensorWidth = ofToString(getSensorWidth());
        std::string sensorHeight = ofToString(getSensorHeight());
        std::string expValue = ofToString(getExposureValue());
        ArvAuto expAuto = getExposureAuto();
        
        std::string expAutoMode = expAuto == 0 ? "ARV_AUTO_OFF" : expAuto == 1 ? "ARV_AUTO_ONCE" : "ARV_AUTO_CONTINUOUS";
        
        if (isInited()) {
            ofDrawBitmapStringHighlight(model, glm::vec2(x,y+0), ofColor::white, ofColor::black);
            ofDrawBitmapStringHighlight("SENSOR: " + sensorWidth + " x " + sensorHeight, glm::vec2(x,y+20));
            ofDrawBitmapStringHighlight("MODE: " + expAutoMode, glm::vec2(x,y+40));
            ofDrawBitmapStringHighlight("EXPOSURE: " + expValue, glm::vec2(x,y+60));
            ofDrawBitmapStringHighlight("CELCIUS: " + temp, glm::vec2(x,y+80));
            ofDrawBitmapStringHighlight("FPS: " + FPS + " / APP: " + appFPS, glm::vec2(x,y+100));
            ofDrawBitmapStringHighlight("MIN/MAX: " + minFPS + " / " + maxFPS, glm::vec2(x,y+120));
        } else {
            ofDrawBitmapStringHighlight("UNINITIALISED.", glm::vec2(x, y+0));
        }
    }

    void Grabber::setPixelFormat(ArvPixelFormat format) {
        targetPixelFormat = format;
    }

    Device & Grabber::getInfo() {
        return info;
    }

    bool Grabber::isInited() {
        return inited;
    }

    bool Grabber::setup( int targetCamera, int targetX, int targetY, int targetWidth, int targetHeight, const char * targetPixelFormat ) {
        stop();
        
        bFrameNew = false;
        
        GError *err = nullptr;
        
        if ( targetCamera >= ofxAravis::ListAllDevices(false).size() ) {
            ofLogError("ofxAravis") << "No camera to open at: " << targetCamera;
            inited = false;
            return inited;
        }
        
        info = GetDeviceInfo( targetCamera );
        camera = arv_camera_new(info.id.c_str(), &err);
        
        if (!camera) {
            ofLogError("ofxAravis") << "No camera could be created at: " << targetCamera;
            inited = false;
            return inited;
        }
        
        arv_camera_get_region(camera, &initX, &initY, &initWidth, &initHeight, &err);
        initPixelFormat = arv_camera_get_pixel_format_as_string( camera, &err );
        
        ofLogNotice("ofxAravis") << "DEFAULTS FROM CAMERA: " << initX << ", " << initY << ", " << initWidth << ", " << initHeight << ", " << initPixelFormat;
        
        // WIDTH & HEIGHT
        
        if (targetWidth != -1 && targetHeight != -1) {
            araWidth = targetWidth;
            araHeight = targetHeight;
            ofLogWarning("ofxAravis") << "WIDTH / HEIGHT: it is recommended to use camera defaults for the capture frame.";
        } else {
            araWidth = initWidth;
            araHeight = initHeight;
        }
        
        // X & Y
        
        if (targetX != -1 && targetY != -1) {
            ofLogWarning("ofxAravis") << "X / Y: it is recommended to use camera defaults for the capture frame.";
            araX = targetX;
            araY = targetY;
        } else {
            araX = initX;
            araY = initY;
        }
        
        // PIXEL FORMAT
        
        if (targetPixelFormat != "") {
            ofLogWarning("ofxAravis") << "PIXEL FORMAT: it is recommended to use camera defaults for pixel format.";
            araPixelFormat = targetPixelFormat;
        } else {
            araPixelFormat = initPixelFormat;
        }
        
        // SENSOR WIDTH & HEIGHT
        
        arv_camera_get_sensor_size(camera, &sensorWidth, &sensorHeight, &err);
        ofLogNotice("ofxAravis") << "SENSOR SIZE: " << sensorWidth << ", " << sensorHeight;
        
        // LIST ALL PIXEL FORMATS
        
        guint numFormats;
        arv_camera_dup_available_pixel_formats(camera, &numFormats, &err);
        const char** formatsChar = arv_camera_dup_available_pixel_formats_as_strings(camera, &numFormats, &err);
        std::string allFormats = "";
        for(int i = 0; i < numFormats; i++) {
            std::string format = std::string(formatsChar[i]);
            formats.push_back(format);
            allFormats = format + " | " + allFormats;
        }
        ofLogNotice("ofxAravis") << "PIXEL FORMATS: " << allFormats;
        // A) SETTING ...
        
        arv_camera_set_region(camera, araX, araY, araWidth, araHeight, &err);
        arv_camera_set_pixel_format_from_string(camera, araPixelFormat, &err);
        
        // B) GETTING ...
        
        arv_camera_get_region(camera, &x, &y, &width, &height, &err);
        pixelFormat = arv_camera_get_pixel_format_as_string(camera, &err);
        
        // C) REPORT ...
        
        ofLogNotice("ofxAravis") << "GRABBER SET TO: " << x << ", " << y << ", " << width << ", " << height << ", " << pixelFormat;
        
        image.allocate(width, height, ofImageType::OF_IMAGE_GRAYSCALE);
        
        auto payload = arv_camera_get_payload(camera, &err);
        
        stream = arv_camera_create_stream(camera, nullptr, nullptr, &err);
        
        if (stream != nullptr) {
            
            // Push 50 buffer in the stream input buffer queue
            for (int i = 0; i < 20; i++)
                arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
            
            //start stream
            arv_camera_start_acquisition(camera, &err);
            
            // Connect the new-buffer signal
            g_signal_connect (stream, "new-buffer", G_CALLBACK(onNewBuffer), this);
            arv_stream_set_emit_signals(stream, TRUE);
            inited = true;
        } else {
            ofLogError("ofxARavis") << "create stream failed";
            inited = false;
        }
        
        return inited;
    }

    int Grabber::getSensorWidth() { return sensorWidth; }
    int Grabber::getSensorHeight() { return sensorHeight; }

    bool Grabber::isInitialized() {
        return camera;
    }

    void Grabber::stop() {
        if (!isInitialized())
            return;
        
        ofLogNotice("ofxAravis") << "Stop";
        
        arv_stream_set_emit_signals(stream, FALSE);
        GError *err = nullptr;
        
        arv_camera_stop_acquisition(camera, &err);
        g_object_unref(stream);
        g_object_unref(camera);
    }

    void Grabber::setExposure(double exposure) {
        if (!isInitialized())
            return;
        GError *err = nullptr;
        
        arv_camera_set_exposure_time_auto(camera, ARV_AUTO_OFF, &err);
        arv_camera_set_exposure_time(camera, exposure, &err);
    }

    void Grabber::draw(int x, int y, int w, int h) {
        if (w == 0) w = width;
        if (h == 0) h = height;
        image.draw(x, y, w, h);
    }

    Grabber::Clock::time_point Grabber::last_frame() {
        mutex.lock();
        auto p = p_last_frame;
        mutex.unlock();
        return p;
    }

    double Grabber::getTemperature() {
        if (!camera) return 0;
        ArvDevice *dev = arv_camera_get_device(camera);
        if (!dev) return 0;
        GError *err = nullptr;
        double val = arv_device_get_float_feature_value(dev, "DeviceTemperature", &err);
        if (err) {
            ofLogError("ofxAravis") << "Could not read temperature " << err->message;
            return 0;
        }
        return val;
    }

}
