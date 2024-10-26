#include "ofxGenicam.h"
#include <arv.h>

namespace ofxGenicam {


	ofJson listDevices(bool print) {

		ofJson devices;

		arv_update_device_list();

		for (int i = 0; i < arv_get_n_devices(); i++) {

			devices[i] = ofJson::array();

			devices[i]["id"] = arv_get_device_id(i);
			devices[i]["physical_id"] = arv_get_device_physical_id(i);
			devices[i]["address"] = arv_get_device_address(i);
			devices[i]["vendor"] = arv_get_device_vendor(i);
			devices[i]["manufacturer_info"] = arv_get_device_manufacturer_info(i);
			devices[i]["model"] = arv_get_device_model(i);
			devices[i]["serial_nbr"] = arv_get_device_serial_nbr(i);
			devices[i]["protocol"] = arv_get_device_protocol(i);
			
		}

		if (print) ofLogNotice("listDevices") << devices.dump(4);

		return devices;
	}

	// ====== CALLBACKS ======

	void Camera::setBufferCallback(BufferCallback callback) {
		bufferCallback = callback;
	}

	void Camera::setErrorCallback(ErrorCallback callback) {
		errorCallback = callback;
	}

	// ====== SETUP ======

	Camera::Camera() {
	}

	bool Camera::open( int index ) {

		ofAddListener(ofEvents().exit, this, &Camera::onAppExit );
		GError* error = nullptr;
		camera = arv_camera_new( arv_get_device_id(index), &error );
		return !handleError(error, "Camera");
	}

	Camera::~Camera() {

		if (isStreaming) {
			GError *err = nullptr;
			arv_camera_abort_acquisition( camera, &err );
			handleError( err, "~Camera" );
			stop();
		}

		g_object_unref(stream);
		g_object_unref(camera);

		ofRemoveListener(ofEvents().exit, this, &Camera::onAppExit);
	}

	void Camera::onAppExit(ofEventArgs& args) {
		stop();
	}

	// ====== UTILITIES ======

	std::string Camera::safeConvertChars(const char* chars) {
		if (!chars) return "N/A";
		return ofToString(chars);
	}

	bool Camera::handleError(GError* err, std::string origin) {
		if (err) {
			ofLogError("ofxAravis") << "ERROR:" << origin << err->message;
			if (errorCallback) errorCallback(origin, err->message);
			g_clear_error(&err);
			return true;
		}
		return false;
	}




}
