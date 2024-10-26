#include "ofxGenicam.h"
#include <arv.h>

namespace ofxGenicam {

	// ====== STREAM ======

	bool Camera::start( int numberOfBuffers ) {

		GError * err = nullptr;

		// PIXEL FORMAT

		pixelFormat = safeConvertChars( arv_camera_get_pixel_format_as_string( camera, &err ) );
		if (handleError( err, "arv_camera_get_pixel_format_as_string" )) return false;

		// PAYLOADS

		auto payload = arv_camera_get_payload(camera, &err);
		if (handleError( err, "arv_camera_get_payload" )) return false;

		// STREAM

		stream = arv_camera_create_stream(camera, nullptr, nullptr, &err);
		if (handleError( err, "arv_camera_create_stream" ) || !stream) {
			g_object_unref( stream );
			return false;
		}

		// BUFFERS

		for (int i = 0; i < numberOfBuffers; i++) {
			arv_stream_push_buffer(stream, arv_buffer_new(payload, nullptr));
		}
			
		arv_camera_start_acquisition(camera, &err);
		if (handleError( err, "arv_camera_start_acquisition" )) {
			g_object_unref( stream );
			return false;
		}
		
		g_signal_connect(stream, "new-buffer", G_CALLBACK(onNewBuffer), this);
		arv_stream_set_emit_signals(stream, true);

        ofLog() << "STARTING";
		return true;
	}

	bool Camera::stop() {

		arv_stream_set_emit_signals(stream, false);
		GError * err = nullptr;
		arv_camera_stop_acquisition(camera, &err);
		return !handleError( err, "arv_camera_stop_acquisition" );
		
	}

	// ====== STREAM ======

	void Camera::onNewBuffer(ArvStream* stream, Camera * instance) {

		ArvBuffer * buffer = arv_stream_try_pop_buffer(stream);
		if (buffer == nullptr) {
			ofLogError("onNewBuffer") << "buffer is nullptr";
			return;
		}

		int status = arv_buffer_get_status(buffer);
		if (status != ARV_BUFFER_STATUS_SUCCESS) {
			ofLogError("onNewBuffer") << "buffer status:" << status;
			arv_stream_push_buffer(stream, buffer);
			return;
		}

		auto width = arv_buffer_get_image_width(buffer);
		auto height = arv_buffer_get_image_height(buffer);
        ArvPixelFormat format = arv_buffer_get_image_pixel_format(buffer);

		uint32_t bitsPerPixel = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(format);
        
        const void* data = arv_buffer_get_data(buffer, nullptr);

		if (data == nullptr) {
			ofLogError("onNewBuffer") << "buffer data is nullptr";
			arv_stream_push_buffer(stream, buffer);
			return;
		}

        if (bitsPerPixel == 8) {
            auto* rawPixels = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data));
            instance->bufferCallback(rawPixels, width, height, bitsPerPixel, instance->pixelFormat);
        } else if (bitsPerPixel >= 12 && bitsPerPixel < 32) {
            auto* rawPixels = const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(data));
            instance->bufferCallback(rawPixels, width, height, bitsPerPixel, instance->pixelFormat);
        } else if (bitsPerPixel == 32) {
            auto* rawPixels = const_cast<uint32_t*>(reinterpret_cast<const uint32_t*>(data));
            instance->bufferCallback(rawPixels, width, height, bitsPerPixel, instance->pixelFormat);
        } else {
            ofLogError("onNewBuffer") << "unsupported bits per pixel: " << bitsPerPixel;
        }

		arv_stream_push_buffer(stream, buffer);
	}



}
