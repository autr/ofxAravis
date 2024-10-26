#include <ofxAravis.h>
#include <opencv2/opencv.hpp>

namespace ofxAravis {

	// RESEARCH THIS:
	//arv_device_set_string_feature_value(Device, "TriggerMode", "On");
	//arv_device_set_string_feature_value(Device, "TriggerSource", "Line1");
	//arv_device_set_string_feature_value(Device, "TriggerActivation", "RisingEdge");
	//arv_device_set_string_feature_value(Device, "AcquisitionMode", "Continuous");
	// https://aravisproject.github.io/aravis/method.Camera.set_trigger.html
	// https://www.flir.eu/support-center/iis/machine-vision/application-note/using-logic-blocks

	ofTexture & Grabber::getTexture() {
		return image.getTexture();
	}
	void Grabber::setBufferCallback(BufferCallback callback) {
		bufferCallback = callback;
	}
	void Grabber::onNewBuffer(ArvStream *stream, Grabber *aravis) {
		ArvBuffer *buffer;
		
		buffer = arv_stream_try_pop_buffer(stream);
		if (buffer != nullptr) {
			if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
				auto w = arv_buffer_get_image_width(buffer);
				auto h = arv_buffer_get_image_height(buffer);
				auto format = arv_buffer_get_image_pixel_format(buffer);
				
//                ofLog() << "FORMAT" << format;
				
				cv::Mat matRgb(h, w, CV_8UC3);
				
//                if (!formatIsFound) {
					// DO THIS
					// for loop over all ARV_PIXEL_FORMATS
					// if (format == ARV_PIXEL_FORMAT_XXXX) ofLog() << "ARV FORMAT" << nameOfARVPixelFormat;
				
//                }
				switch (format) {
					case ARV_PIXEL_FORMAT_BAYER_RG_8: {
						cv::Mat matBayer(h, w, CV_8UC1, const_cast<void *>(arv_buffer_get_data(buffer, nullptr)));
						cv::cvtColor(matBayer, matRgb, CV_BayerRG2BGR);
					}
						break;

					case ARV_PIXEL_FORMAT_BAYER_GB_8: {
						cv::Mat matBayer(h, w, CV_8UC1, const_cast<void *>(arv_buffer_get_data(buffer, nullptr)));
						cv::cvtColor(matBayer, matRgb, CV_BayerGB2BGR);
					}
						break;

					default:
						ofLogError("ofxAravis") << "Unknown pixel format";
				}

				aravis->setPixels(matRgb);
				
				if (aravis->bufferCallback) aravis->bufferCallback(matRgb);
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
		
		float time = ofGetElapsedTimef();
		fpsTimeElapsed = time - previousTimestamp;
		previousTimestamp = time;
		totalFrames = totalFrames + 1;
	}

	void HandleError( GError * err ) {
		if ( err != NULL ) {
			ofLogError("ofxAravis") << "ERROR:" << err->message;
		}
	}

	std::vector<std::string> ArrayToVector( const char ** array, int length ) {
		std::vector<std::string> stringVector(array, array + length);
		return stringVector;
	}


	// ------- EXPOSURE TIME -------

	bool Grabber::hasExposureTime() {
		GError *err = nullptr;
		bool has = arv_camera_is_exposure_time_available( camera, &err );
		HandleError( err );
		return has;
	}

	void Grabber::setExposureTime( double value ) {
		GError *err = nullptr;
		arv_camera_set_exposure_time(camera, value, &err);
		HandleError( err );
	}

	double Grabber::getExposureTime() {
		GError *err = nullptr;
		double value = arv_camera_get_exposure_time(camera, &err);
		HandleError( err );
		return value;
	}


	Bounds Grabber::getExposureBounds() {
		GError *err = nullptr;
		Bounds minMax;
		arv_camera_get_exposure_time_bounds(camera, &minMax.max, &minMax.min, &err);
		HandleError( err );
		return minMax;
	}


	// ------- EXPOSURE TIME AUTO -------

	bool Grabber::hasExposureTimeAuto() {
		GError *err = nullptr;
		bool has = arv_camera_is_exposure_auto_available( camera, &err );
		HandleError( err );
		return has;
	}

	void Grabber::setExposureTimeAuto( ArvAuto mode ) {
		GError *err = nullptr;
		arv_camera_set_exposure_time_auto(camera, mode, &err);
		HandleError( err );
	}

	ArvAuto Grabber::getExposureTimeAuto() {
		GError *err = nullptr;
		ArvAuto mode = arv_camera_get_exposure_time_auto(camera, &err);
		HandleError( err );
		return mode;
	}

	// ------- TRIGGER MODE -------

	void Grabber::setTriggerMode( std::string key ) {
		ofLog() << "setting trigger source:" << key;
		GError *err = nullptr;
		const char * keyChar = key.c_str();
		arv_camera_set_trigger( camera, keyChar, &err );
		HandleError( err );
	}

	vector<std::string> Grabber::getAvailableTriggerModes( ) {
		guint numValues;
		GError *err = nullptr;
		const char ** array = arv_camera_dup_available_triggers( camera, &numValues, &err);
		vector<std::string> strings = ArrayToVector( array, numValues );
		for (int i = 0; i < strings.size(); i++) {
			ofLogNotice("ofxAravis") << "TRIGGER MODE:" << i << strings[i];
		}
		return strings;
	}

	// ------- TRIGGER SOURCE -------

	void Grabber::setTriggerSource( std::string key ) {
		ofLog() << "setting trigger source:" << key;
		GError *err = nullptr;
		const char * keyChar = key.c_str();
		arv_camera_set_trigger_source( camera, keyChar, &err );
		HandleError( err );
	}

	std::string Grabber::getTriggerSource( ) {
		GError *err = nullptr;
		const char * modeChar = arv_camera_get_trigger_source( camera, &err );
		HandleError( err );
		std::string modeStr( modeChar );
		return modeStr;
	}

	vector<std::string> Grabber::getAvailableTriggerSources( ) {
		guint numValues;
		GError *err = nullptr;
		const char ** array = arv_camera_dup_available_trigger_sources( camera, &numValues, &err);
		vector<std::string> strings = ArrayToVector( array, numValues);
		for (int i = 0; i < strings.size(); i++) {
			ofLogNotice("ofxAravis") << "TRIGGER SOURCE:" << i << strings[i];
		}
		return strings;
	}


	// ------- ENUMERATIONS -------
	

	vector<std::string> Grabber::getAvailableEnumerations( std::string key ) {
		guint numValues;
		GError *err = nullptr;
		const char ** array = arv_camera_dup_available_enumerations_as_strings( camera, key.c_str(), &numValues, &err);
		vector<std::string> strings = ArrayToVector( array, numValues );
//        for (int i = 0; i < strings.size(); i++) {
//            ofLogNotice("ofxAravis") << "ENUMERATE:" << key << i << strings[i];
//        }
		return strings;
	}


	// ------- FEATURES -------

    std::string Grabber::safeConvertChars( const char * chars ) {
    	if (!chars) return "N/A";
    	return ofToString(chars);
    }

    ofJson Grabber::listAllFeatures() {

        ofJson features;

        ArvDevice * device;
        device = arv_camera_get_device( camera );

        ArvGc * genicam;
        genicam = arv_device_get_genicam( device );
        
        ofLog() << "listing all features...";
        loopThroughFeatures( features, genicam, "Root", 0 );
    
        ofLog() << features;
        
        return features;
    }

	void Grabber::loopThroughFeatures( ofJson & features, ArvGc * genicam, const char * feature, int level ) {

		ArvGcNode * node;
		node = arv_gc_get_node( genicam, feature );

		bool isFeature = ARV_IS_GC_FEATURE_NODE(node);
		bool isImplemented = arv_gc_feature_node_is_implemented(ARV_GC_FEATURE_NODE(node), NULL);

		if ( isFeature && isImplemented ) {

			processFeatureNode( features, ARV_GC_FEATURE_NODE(node), level );

			if ( ARV_IS_GC_CATEGORY(node) ) {

				const GSList * featuresList;
				const GSList * iter;

                features["children"] = ofJson::array();

                featuresList = arv_gc_category_get_features( ARV_GC_CATEGORY(node) );
				int i = 0;

				for (iter = featuresList; iter != NULL; iter = iter->next) {
	                const char * featureName = static_cast<const char *>(iter->data);
                    ofJson childJson;
					loopThroughFeatures( features["children"][i], genicam, featureName, level + 1 );
                    i += 1;
				}
			}
		}


	}


	void Grabber::processFeatureNode( ofJson & entry, ArvGcFeatureNode * node, int level ) {

			std::string nodeType = safeConvertChars( arv_dom_node_get_node_name (ARV_DOM_NODE (node)) );
			std::string featureName = safeConvertChars( arv_gc_feature_node_get_name (node) );
        
            entry["nodeType"] = nodeType;
            entry["featureName"] = featureName;

			bool isEnumeration = ARV_IS_GC_ENUMERATION( node );
        
            ofLog() << "======" << nodeType+":"+featureName << "======";

            if (arv_gc_feature_node_is_available (node, NULL)) {

                bool isSelector = ARV_IS_GC_SELECTOR(node) && arv_gc_selector_is_selector( ARV_GC_SELECTOR( node ) );

                char *value = NULL;
                GError *error = NULL;

                std::string accessMode = ofToString( arv_gc_access_mode_to_string( arv_gc_feature_node_get_actual_access_mode(node) ));
                
                entry["accessMode"] = accessMode;

                if (true) { // list_mode == ARV_TOOL_LIST_MODE_VALUES) {

                    const char * unit;

                    if (ARV_IS_GC_STRING(node) || isEnumeration) {

                        std::string value = arv_gc_string_get_value( ARV_GC_STRING(node), &error);
                        
                        entry["value"] = value;
                        ofLog() << "string:" << value;

                    } else if (ARV_IS_GC_INTEGER (node)) {

                        if (isEnumeration) {

                            std::string value = arv_gc_string_get_value( ARV_GC_STRING(node), &error );
                            
                            entry["value"] = value;
                            ofLog() << "enum:" << value;

                        } else {

                            std::string unit = safeConvertChars( arv_gc_integer_get_unit( ARV_GC_INTEGER(node) ) );
                            int value = arv_gc_integer_get_value (ARV_GC_INTEGER (node), &error);
                            
                            entry["value"] = value;
                            if (unit != "N/A") entry["unit"] = unit;
                            
                            ofLog() << "enum + unit:" << value << " / " << unit;

                        }
                    } else if (ARV_IS_GC_FLOAT (node)) {

                        std::string unit = safeConvertChars( arv_gc_float_get_unit( ARV_GC_FLOAT(node) ) );
                        float value = arv_gc_float_get_value( ARV_GC_FLOAT (node), &error);
                        
                        entry["value"] = value;
                        if (unit != "N/A") entry["unit"] = unit;
                        
                        ofLog() << "float + unit:" << value << " / " + unit;

                    } else if (ARV_IS_GC_BOOLEAN (node)) {

                        bool value = arv_gc_boolean_get_value( ARV_GC_BOOLEAN(node), &error );
                        entry["value"] = value;
                        
                        ofLog() << "bool:" << (value ? "true" : "false");

                    }
                }


                if (isSelector) {

                        const GSList * iter;
                        vector<std::string> options;
                        int i = 0;
                    
                        entry["options"] = ofJson::array();

                        // Get the list of selected feature nodes
                        const GSList *selectedFeatures = arv_gc_selector_get_selected_features(ARV_GC_SELECTOR(node));

                        for (iter = selectedFeatures; iter != NULL; iter = iter->next) {
                            // Cast iter->data to an ArvGcFeatureNode*
                            ArvGcFeatureNode * featureNode = ARV_GC_FEATURE_NODE(iter->data);
                            std::string option = safeConvertChars(arv_gc_feature_node_get_name(featureNode));
                            
                            entry["enum"][i] = option;
                            ofLog() << "--->" << i << option;
                            options.push_back(option);
                            i += 1;
                        }

                }
			}


			std::string description = safeConvertChars( arv_gc_feature_node_get_description(node) );
        
            entry["description"] = description;
			ofLog() << "description:" << description;
        
            if (isEnumeration) {
                const GSList *childs;
                const GSList *iter;

                // Get the list of entries for the enumeration
                childs = arv_gc_enumeration_get_entries(ARV_GC_ENUMERATION(node));

                int i = 0;
                entry["enum"] = ofJson::array();

                for (iter = childs; iter != NULL; iter = iter->next) {
                    // Cast iter->data to ArvGcFeatureNode*
                    ArvGcFeatureNode *featureNode = ARV_GC_FEATURE_NODE(iter->data);
                    
                    if (arv_gc_feature_node_is_implemented(featureNode, NULL)) {
                        // Cast iter->data to ArvDomNode* for getting the node name
                        ArvDomNode *domNode = ARV_DOM_NODE(iter->data);

                        std::string childNodeType = safeConvertChars(arv_dom_node_get_node_name(domNode));
                        std::string childFeatureName = safeConvertChars(arv_gc_feature_node_get_name(featureNode));
                        bool childFeatureIsAvailable = arv_gc_feature_node_is_available(featureNode, NULL);
                        
                        ofJson childEntry;
                        childEntry["nodeType"] = childNodeType;
                        childEntry["featureName"] = childFeatureName;
                        childEntry["isAvailable"] = childFeatureIsAvailable;
                        
                        entry["enum"][i] = childEntry;

                        ofLog() << "--->" << i << " " << childNodeType << " " << childFeatureName << " " << (childFeatureIsAvailable ? "Available" : "Not available");

                        i += 1;
                    }
                }
            }

	}


	void Grabber::setFeatureString( std::string key, std::string value ) {
			ofLog() << "setting string feature:" << key << value;
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			const char * valChar = value.c_str();
			arv_camera_set_string( camera, keyChar, valChar, &err );
			HandleError( err );
	}

	void Grabber::executeCommand( std::string command ) {
		const char * commandChars = command.c_str();
		GError *err = nullptr;
		ofLog() << "executing command:" << command;
		arv_camera_execute_command( camera, commandChars, &err);
		HandleError( err );
	}
	std::string Grabber::getFeatureString( std::string key ) {
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			auto res = arv_camera_get_string( camera, keyChar, &err );
			std::string value = (res == NULL) ? "ERROR" : std::string(res);
			HandleError( err );
			return value;
	}

	void Grabber::setFeatureBoolean( std::string key, bool value ) {
			ofLog() << "setting bool feature:" << key << value;
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			arv_camera_set_boolean( camera, keyChar, value, &err );
			HandleError( err );
	}
	bool Grabber::getFeatureBoolean( std::string key ) {
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			auto res = arv_camera_get_boolean( camera, keyChar, &err );
			bool value = (res == NULL) ? false : bool(res);
			HandleError( err );
			return value;
	}

	void Grabber::setFeatureInteger( std::string key, int value ) {
			ofLog() << "setting integer feature:" << key << value;
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			arv_camera_set_integer( camera, keyChar, gint64(value), &err );
			HandleError( err );
	}
	int Grabber::getFeatureInteger( std::string key ) {
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			auto res = arv_camera_get_integer( camera, keyChar, &err );
			int value = (res == NULL) ? -1 : int(res);
			HandleError( err );
			return value;
	}
	
	void Grabber::setFeatureFloat( std::string key, int value ) {
			ofLog() << "setting float feature:" << key << value;
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			arv_camera_set_float( camera, keyChar, gint64(value), &err );
			HandleError( err );
	}
   float Grabber::getFeatureFloat( std::string key ) {
			GError *err = nullptr;
			const char * keyChar = key.c_str();
			auto res = arv_camera_get_float( camera, keyChar, &err );
			float value = (res == NULL) ? -1.0 : float(res);
			HandleError( err );
			return value;
	}

	// ------- FPS -------

	void Grabber::setFPS( double fps ) {
		GError *err = nullptr;
		arv_camera_set_frame_rate(camera, fps, &err);
		HandleError( err );
	}
	double Grabber::getFPS() {
		GError *err = nullptr;
		double fps = arv_camera_get_frame_rate(camera, &err);
		HandleError( err );
		return fps;
	}
	double Grabber::getMinFPS() {
		GError *err = nullptr;
		double min;
		double max;
		arv_camera_get_frame_rate_bounds( camera, &min, &max, &err);
		HandleError( err );
		return min;
	}
	double Grabber::getMaxFPS() {
		GError *err = nullptr;
		double min;
		double max;
		arv_camera_get_frame_rate_bounds( camera, &min, &max, &err);
		HandleError( err );
		return max;
	}
	float Grabber::getActualFPS() {
		return float( 1.0 / fpsTimeElapsed );
	}

	// ------- FORMAT -------

	void Grabber::setPixelFormat( std::string format ) {
		GError *err = nullptr;
		const char * formatChar = format.c_str();
		arv_camera_set_pixel_format_from_string(camera, formatChar, &err);
		HandleError( err );
	}
	std::string Grabber::getPixelFormat() {
		GError *err = nullptr;
		auto res = arv_camera_get_pixel_format_as_string(camera, &err );
		std::string value = (res == NULL) ? "ERROR" : std::string(res);
		HandleError( err );
		return value;
	}

	bool Grabber::update() {
		if (bFrameNew) {
			bFrameNew = false;
			mutex.lock();
			image.setFromPixels(mat.data, width, height, ofImageType::OF_IMAGE_COLOR);
			mutex.unlock();
			return true;
		} else {
			return false;
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
		if (devices.size() <= 0) ofLogNotice("ofxAravis") << "NO DEVICES.";
		return devices;
	}

	Grabber::Grabber() {
		ofLogNotice("ofxAravis") << "created";
		p_last_frame = Clock::now();
		ofAddListener(ofEvents().exit, this, &Grabber::onAppExit);
	}

	void Grabber::onAppExit(ofEventArgs& args) {
		ofLogNotice("ofxAravis") << "on app exit";
		stop();
	}

	Grabber::~Grabber() {
		ofLogNotice("ofxAravis") << "destroying";
		stop();
	}

	void Grabber::drawInfo( int x, int y ) {
		
		glm::vec2 center = glm::vec2( ofGetWidth()/2, ofGetHeight()/2 );
		
		std::string model = getInfo().model;
		std::string FPS = ofToString(getFPS());
		std::string minFPS = ofToString(getMinFPS());
		std::string maxFPS = ofToString(getMaxFPS());
		std::string appFPS = ofToString(int(ofGetFrameRate()));
//        std::string temp = ofToString(getTemperature());
		std::string currentFormat = getPixelFormat();
		std::string sensorWidth = ofToString(getSensorWidth());
		std::string sensorHeight = ofToString(getSensorHeight());
		std::string actualFPS = ofToString(getActualFPS());
		std::string gainAuto = ofToString(getFeatureString( "GainAuto" ));
		
		std::string expValue = ofToString(getExposureTime());
		ArvAuto expAuto = getExposureTimeAuto();
		std::string expAutoMode = expAuto == 0 ? "ARV_AUTO_OFF" : expAuto == 1 ? "ARV_AUTO_ONCE" : "ARV_AUTO_CONTINUOUS";
		
		Bounds exposureBounds = getExposureBounds();
		
		std::string triggerSource = getTriggerSource();
		if (availableTriggerModes.size() == 0) {
			availableTriggerModes = getAvailableTriggerModes();
		}
		if (availableTriggerSources.size() == 0) {
			availableTriggerSources = getAvailableTriggerSources();
		}
		
		if (isInited()) {
			ofDrawBitmapStringHighlight(model, glm::vec2(x,y+0), ofColor::white, ofColor::black);
			ofDrawBitmapStringHighlight("SENSOR: " + sensorWidth + " x " + sensorHeight, glm::vec2(x,y+20));
			ofDrawBitmapStringHighlight("MODE: " + expAutoMode, glm::vec2(x,y+40));
			ofDrawBitmapStringHighlight("EXPOSURE: " + expValue, glm::vec2(x,y+60));
			//            ofDrawBitmapStringHighlight("CELCIUS: " + temp, glm::vec2(x,y+80));
			ofDrawBitmapStringHighlight("FORMAT: " + currentFormat, glm::vec2(x,y+80));
			ofDrawBitmapStringHighlight("FPS: " + FPS + " / APP: " + appFPS, glm::vec2(x,y+100));
			ofDrawBitmapStringHighlight("FPS MIN / MAX: " + minFPS + " / " + maxFPS, glm::vec2(x,y+120));
			ofDrawBitmapStringHighlight("ACTUAL FPS: " + actualFPS, glm::vec2(x,y+140));
			ofDrawBitmapStringHighlight("TRIGGER SOURCE: " + triggerSource, glm::vec2(x,y+160));
			ofDrawBitmapStringHighlight("EXPOSURE BOUNDS: " + ofToString(exposureBounds.min) + " / " + ofToString(exposureBounds.max), glm::vec2(x,y+180));
			ofDrawBitmapStringHighlight("TOTAL FRAMES: " + ofToString(totalFrames), glm::vec2(x,y+200));
			ofDrawBitmapStringHighlight("GAIN AUTO: " + gainAuto, glm::vec2(x,y+220));
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
		totalFrames = 0;
		previousTimestamp = ofGetElapsedTimef();
		fpsTimeElapsed = 0;
		
		GError *err = nullptr;
		
		if ( targetCamera >= ofxAravis::ListAllDevices(false).size() ) {
			ofLogError("ofxAravis") << "No camera to open at: " << targetCamera;
			inited = false;
			return inited;
		}
		
		info = GetDeviceInfo( targetCamera );
		camera = arv_camera_new(info.id.c_str(), &err);
		
		HandleError( err );
		
		if (!camera) {
			ofLogError("ofxAravis") << "No camera could be created at: " << targetCamera;
			inited = false;
			return inited;
		}
		
		arv_camera_get_region(camera, &initX, &initY, &initWidth, &initHeight, &err);
		HandleError( err );
		initPixelFormat = arv_camera_get_pixel_format_as_string( camera, &err );
		HandleError( err );
		
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
		HandleError( err );
		ofLogNotice("ofxAravis") << "SENSOR SIZE: " << sensorWidth << ", " << sensorHeight;
		
		// LIST ALL PIXEL FORMATS
		
		guint numFormats;
		arv_camera_dup_available_pixel_formats(camera, &numFormats, &err);
		HandleError( err );
		const char** formatsChar = arv_camera_dup_available_pixel_formats_as_strings(camera, &numFormats, &err);
		HandleError( err );
		std::string allFormats = "";
		for(int i = 0; i < numFormats; i++) {
			std::string format = std::string(formatsChar[i]);
			formats.push_back(format);
			allFormats = format + " | " + allFormats;
		}
		
		// A) SETTING ...
		
		arv_camera_set_region(camera, araX, araY, araWidth, araHeight, &err);
		HandleError( err );
		arv_camera_set_pixel_format_from_string(camera, araPixelFormat, &err);
		HandleError( err );
		
		// B) GETTING ...
		
		arv_camera_get_region(camera, &x, &y, &width, &height, &err);
		HandleError( err );
		pixelFormat = arv_camera_get_pixel_format_as_string(camera, &err);
		if (pixelFormat == NULL) {
			ofLogError("ofxAravis") << "pixelFormat reports NULL";
			pixelFormat = "NULL";
		}
		HandleError( err );
		
		// C) REPORT ...
		
		ofLogNotice("ofxAravis") << "GRABBER SET TO: " << x << ", " << y << ", " << width << ", " << height << ", " << pixelFormat;
		
		image.allocate(width, height, ofImageType::OF_IMAGE_GRAYSCALE);
		
		auto payload = arv_camera_get_payload(camera, &err);
		HandleError( err );
		
		// ------ STREAMS ------
		
		stream = arv_camera_create_stream(camera, nullptr, nullptr, &err);
		HandleError( err );
		
		int numberOfBuffers = 100;
		
		if (stream != nullptr) {
			
			// Push 50 buffer in the stream input buffer queue
			for (int i = 0; i < numberOfBuffers; i++) arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
			
			//start stream
			arv_camera_start_acquisition(camera, &err);
			HandleError( err );
			
			// Connect the new-buffer signal
			g_signal_connect(stream, "new-buffer", G_CALLBACK(onNewBuffer), this);
			arv_stream_set_emit_signals(stream, TRUE);
			inited = true;
		} else {
			ofLogError("ofxARavis") << "create stream failed";
			inited = false;
		}
		
		return inited;
	}

	int Grabber::getWidth() {
		return initWidth;
	}
	int Grabber::getHeight() {
		return initHeight;
	}

	int Grabber::getSensorWidth() { return sensorWidth; }
	int Grabber::getSensorHeight() { return sensorHeight; }

	bool Grabber::isInitialized() {
		return camera;
	}

	void Grabber::stop() {
		if (!isInitialized()) return;
		
		ofLogNotice("ofxAravis") << "stopping...";
		
		arv_stream_set_emit_signals(stream, FALSE);
		GError *err = nullptr;
		arv_camera_stop_acquisition(camera, &err);
		HandleError( err );
		g_object_unref(stream);
		g_object_unref(camera);
		ofLogNotice("ofxAravis") << "stopped!";
	}

	void Grabber::setExposure(double exposure) {
		if (!isInitialized())
			return;
		GError *err = nullptr;
		
		arv_camera_set_exposure_time_auto(camera, ARV_AUTO_OFF, &err);
		HandleError( err );
		arv_camera_set_exposure_time(camera, exposure, &err);
		HandleError( err );
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
		ArvDevice * dev = arv_camera_get_device(camera);
		if (!dev) return 0;
		GError *err = nullptr;
		double val = arv_device_get_float_feature_value(dev, "DeviceTemperature", &err);
		HandleError( err );
		return val;
	}


	std::string Grabber::getGenicamXML() {
		const char *genicam_xml;
		size_t size;
		
		ArvDevice *dev = arv_camera_get_device(camera);
		if (!dev) {
			ofLogError("ofxAravis") << "Could not get device";
			return "";
		}
		genicam_xml = arv_device_get_genicam_xml(dev, &size);
		if (!genicam_xml) {
			ofLogError("ofxAravis") << "Failed to retrieve GenICam XML";
			return "";
		}
		return std::string(genicam_xml);

	}

}
