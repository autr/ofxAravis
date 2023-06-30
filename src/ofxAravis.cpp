#include <ofxAravis.h>
#include <opencv2/opencv.hpp>

namespace ofxAravis {

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

    void Grabber::update() {
        if (bFrameNew) {
            bFrameNew = false;
            mutex.lock();
            image.setFromPixels(mat.data, width, height, ofImageType::OF_IMAGE_COLOR);
            mutex.unlock();
        }
    }

    std::vector<Device> & Grabber::listDevices(){
        arv_update_device_list();
        devices.clear();
        for (int i = 0; i < arv_get_n_devices(); i++) {
            
            ofxAravis::Device device;
            
            device.id = arv_get_device_id(i);
            device.physical_id = arv_get_device_physical_id(i);
            device.address = arv_get_device_address(i);
            device.vendor = arv_get_device_vendor(i);
            device.manufacturer_info = arv_get_device_manufacturer_info(i);
            device.model = arv_get_device_model(i);
            device.serial_nbr = arv_get_device_serial_nbr(i);
            device.protocol = arv_get_device_protocol(i);

            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "id: " + device.id;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "physical_id: " + device.physical_id;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "address: " + device.address;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "vendor: " + device.vendor;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "manufacturer_info :" + device.manufacturer_info;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "model: " + device.model;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "serial_nbr: " + device.serial_nbr;
            ofLogNotice("ofxAravis") << "DEVICE " + ofToString(i) + " " << "protocol: " + device.protocol;


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

    void Grabber::setPixelFormat(ArvPixelFormat format) {
        targetPixelFormat = format;
    }

    bool Grabber::setup( int targetX, int targetY, int targetWidth, int targetHeight, const char * targetPixelFormat ) {
        stop();
        
        bFrameNew = false;
        
        GError *err = nullptr;
        
        camera = arv_camera_new(nullptr, &err);
        
        if (!camera) {
            ofLogError("ofxAravis") << "No camera found";
            return false;
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
        } else {
            ofLogError("ofxARavis") << "create stream failed";
        }
        return true;
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
