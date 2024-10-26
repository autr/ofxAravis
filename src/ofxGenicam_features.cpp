#include "ofxGenicam.h"
#include <arv.h>

namespace ofxGenicam {


	// ====== PROCESS FEATURES ======

	void Camera::loopThroughFeatures(ofJson& features, ArvGc* genicam, const char* feature, int level) {

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

	void Camera::processFeatureNode(ofJson& entry, ArvGcFeatureNode* node, int level) {
		
		bool debug = false;

		std::string nodeType = safeConvertChars( arv_dom_node_get_node_name (ARV_DOM_NODE (node)) );
		std::string featureName = safeConvertChars( arv_gc_feature_node_get_name (node) );
	
		entry["nodeType"] = nodeType;
		entry["featureName"] = featureName;

		bool isEnumeration = ARV_IS_GC_ENUMERATION( node );
	
		if (debug) ofLogNotice("processFeatureNode") << "======" << nodeType+":"+featureName << "======";

		if (arv_gc_feature_node_is_available (node, NULL)) {

			bool isSelector = ARV_IS_GC_SELECTOR(node) && arv_gc_selector_is_selector( ARV_GC_SELECTOR( node ) );

			char *value = NULL;
			GError *error = NULL;
			const char * unit;

			std::string accessMode = safeConvertChars( arv_gc_access_mode_to_string( arv_gc_feature_node_get_actual_access_mode(node) ));
			
			entry["accessMode"] = accessMode;


			if (ARV_IS_GC_STRING(node) || isEnumeration) {

				std::string value = arv_gc_string_get_value( ARV_GC_STRING(node), &error);
				
				entry["value"] = value;
				if (debug) ofLogNotice("processFeatureNode") << "string:" << value;

			} else if (ARV_IS_GC_INTEGER (node)) {

				if (isEnumeration) {

					std::string value = arv_gc_string_get_value( ARV_GC_STRING(node), &error );
					
					entry["value"] = value;
					if (debug) ofLogNotice("processFeatureNode") << "enum:" << value;

				} else {

					int value = arv_gc_integer_get_value ( ARV_GC_INTEGER(node), &error);
					int min = arv_gc_integer_get_min( ARV_GC_INTEGER(node), &error);
					int max = arv_gc_integer_get_max( ARV_GC_INTEGER(node), &error);
					float increment = arv_gc_integer_get_inc( ARV_GC_INTEGER(node), &error);
					std::string unit = safeConvertChars( arv_gc_integer_get_unit( ARV_GC_INTEGER(node) ) );

					entry["value"] = value;
					entry["min"] = min;
					entry["max"] = max;
					entry["increment"] = increment;
					if (unit != "N/A") entry["unit"] = unit;
					
					if (debug) ofLogNotice("processFeatureNode") << "enum + unit:" << value << " / " << unit;

				}
			} else if (ARV_IS_GC_FLOAT (node)) {

				float value = arv_gc_float_get_value( ARV_GC_FLOAT(node), &error);
				float min = arv_gc_float_get_min( ARV_GC_FLOAT(node), &error);
				float max = arv_gc_float_get_max( ARV_GC_FLOAT(node), &error);
				float increment = arv_gc_float_get_inc( ARV_GC_FLOAT(node), &error);
				std::string unit = safeConvertChars( arv_gc_float_get_unit( ARV_GC_FLOAT(node) ) );
				
				entry["value"] = value;
				entry["min"] = min;
				entry["max"] = max;
				entry["increment"] = increment;
				if (unit != "N/A") entry["unit"] = unit;
				
				if (debug) ofLogNotice("processFeatureNode") << "float + unit:" << value << " / " + unit;

			} else if (ARV_IS_GC_BOOLEAN (node)) {

				bool value = arv_gc_boolean_get_value( ARV_GC_BOOLEAN(node), &error );
				entry["value"] = value;
				
				if (debug) ofLogNotice("processFeatureNode") << "bool:" << (value ? "true" : "false");

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
						
						entry["options"][i] = option;
						if (debug) ofLogNotice("processFeatureNode") << "--->" << i << option;
						options.push_back(option);
						i += 1;
					}

			}
		}


		std::string description = safeConvertChars( arv_gc_feature_node_get_description(node) );
	
		entry["description"] = description;

		if (debug) ofLogNotice("processFeatureNode") << "description:" << description;
	
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

					if (debug) ofLogNotice("processFeatureNode") << "--->" << i << " " << childNodeType << " " << childFeatureName << " " << (childFeatureIsAvailable ? "Available" : "Not available");

					i += 1;
				}
			}
		}
	}


	// ====== LIST FEATURES ======

	ofJson Camera::listAllFeatures( bool print ) {

		ofJson features;

		ArvDevice * device;
		device = arv_camera_get_device( camera );

		ArvGc * genicam;
		genicam = arv_device_get_genicam( device );
		
		loopThroughFeatures( features, genicam, "Root", 0 );
	
	   if (print) ofLogNotice("listAllFeatures") << features.dump(4);
		
		return features;
	}

	std::string Camera::getGenicamXML() {
        
        const char * genicamXML;
        size_t size;
        
        ArvDevice * dev = arv_camera_get_device(camera);
        if (!dev) {
            ofLogError("ofxAravis") << "Could not get device";
            return "";
        }
        genicamXML = arv_device_get_genicam_xml(dev, &size);
        if (!genicamXML) {
            ofLogError("ofxAravis") << "Failed to retrieve GenICam XML";
            return "";
        }
        return std::string(genicamXML);
	}

	// ====== GETTERS & SETTERS ======

	bool Camera::setStr(std::string key, std::string value) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		const char * valChar = value.c_str();
		arv_camera_set_string( camera, keyChar, valChar, &err );
		return !handleError( err, "setStr" );

	}

	std::string Camera::getStr(std::string key) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		auto res = arv_camera_get_string( camera, keyChar, &err );
		std::string value = (res == NULL) ? "N/A" : std::string(res);
		if (res == NULL) ofLogError("getStr") << "ERROR:" << key;
		handleError( err, "getStr" );
		return value;

	}

	bool Camera::setBool(std::string key, bool value) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		arv_camera_set_boolean( camera, keyChar, value, &err );
		return !handleError( err, "setBool" );

	}

	bool Camera::getBool(std::string key) {

		GError *err = nullptr;
		const char * keyChar = key.c_str();
		auto res = arv_camera_get_boolean( camera, keyChar, &err );
		bool value = (res == NULL) ? false : bool(res);
		if (res == NULL) ofLogError("getBool") << "ERROR:" << key;
		handleError( err, "getBool" );
		return value;
	}

    bool Camera::setInt(std::string key, int value) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		arv_camera_set_integer( camera, keyChar, gint64(value), &err );
		return !handleError( err, "setInt" );

	}

	int Camera::getInt(std::string key) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		auto res = arv_camera_get_integer( camera, keyChar, &err );
		int value = (res == NULL) ? 0 : int(res);
		if (res == NULL) ofLogError("getInt") << "ERROR:" << key;
		handleError( err, "getInt" );
		return value;

	}

    bool Camera::setFloat(std::string key, float value) {

		GError *err = nullptr;
		const char * keyChar = key.c_str();
		arv_camera_set_float( camera, keyChar, gint64(value), &err );
		return !handleError( err, "setFloat" );

	}

	float Camera::getFloat(std::string key) {

		GError * err = nullptr;
		const char * keyChar = key.c_str();
		auto res = arv_camera_get_float( camera, keyChar, &err );
		float value = (res == NULL) ? 0.0 : float(res);
		if (res == NULL) ofLogError("getFloat") << "ERROR:" << key;
		handleError( err, "getFloat" );
		return value;
		
	}

    bool Camera::executeCommand(std::string command) {

		const char * commandChars = command.c_str();
		GError *err = nullptr;
		ofLogNotice("executeCommand") << command;
		arv_camera_execute_command( camera, commandChars, &err);
		return !handleError( err, "executeCommand" );

	}

}
