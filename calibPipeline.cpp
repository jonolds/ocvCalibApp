#include "calibPipeline.hpp"
#include <opencv2/highgui.hpp>
#include <stdexcept>

using namespace calib;

#define CAP_DELAY 10

cv::Size CalibPipeline::getCameraResolution() {
	mCapture.set(cv::CAP_PROP_FRAME_WIDTH, 10000);
	mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, 10000);
	int w = (int)mCapture.get(cv::CAP_PROP_FRAME_WIDTH);
	int h = (int)mCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
	return cv::Size(w, h);
}

PipelineExitStatus CalibPipeline::start(std::vector<cv::Ptr<FrameProcessor> > processors) {
	if (mCaptureParams.source == Camera && !mCapture.isOpened()) {
		mCapture.open(mCaptureParams.camID);
		cv::Size maxRes = getCameraResolution();
		cv::Size neededRes = mCaptureParams.cameraResolution;

		if (maxRes.width < neededRes.width) {
			double aR = (double)maxRes.width / maxRes.height;
			mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.width);
			mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.width / aR);
		} else if (maxRes.height < neededRes.height) {
			double aR = (double)maxRes.width / maxRes.height;
			mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.height);
			mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.height*aR);
		} else {
			mCapture.set(cv::CAP_PROP_FRAME_HEIGHT, neededRes.height);
			mCapture.set(cv::CAP_PROP_FRAME_WIDTH, neededRes.width);
		}
		mCapture.set(cv::CAP_PROP_AUTOFOCUS, 0);
	} else if (mCaptureParams.source == File && !mCapture.isOpened())
		mCapture.open(mCaptureParams.videoFileName);
	mImageSize = cv::Size((int)mCapture.get(cv::CAP_PROP_FRAME_WIDTH), (int)mCapture.get(cv::CAP_PROP_FRAME_HEIGHT));

	if (!mCapture.isOpened())
		throw std::runtime_error("Unable to open video source");

	cv::Mat frame, processedFrame;
	while (mCapture.grab()) {
		mCapture.retrieve(frame);
		if (mCaptureParams.flipVertical)
			cv::flip(frame, frame, -1);

		frame.copyTo(processedFrame);
		for (std::vector<cv::Ptr<FrameProcessor> >::iterator it = processors.begin(); it != processors.end(); ++it)
			processedFrame = (*it)->processFrame(processedFrame);
		cv::imshow(mainWindowName, processedFrame);
		char key = (char)cv::waitKey(CAP_DELAY);

		if (key == 27) // esc
			return Finished;
		if (key == 114) // r
			return DeleteLastFrame;
		if (key == 100) // d
			return DeleteAllFrames;
		if (key == 115) // s
			return SaveCurrentData;
		if (key == 117) // u
			return SwitchUndistort;
		if (key == 118) // v
			return SwitchVisualisation;

		for (std::vector<cv::Ptr<FrameProcessor> >::iterator it = processors.begin(); it != processors.end(); ++it)
			if ((*it)->isProcessed())
				return Calibrate;
	}

	return Finished;
}