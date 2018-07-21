#ifndef FRAME_PROCESSOR_HPP
#define FRAME_PROCESSOR_HPP
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "calibCommon.hpp"
#include "calibController.hpp"

namespace calib {
	class FrameProcessor {
	public:
		virtual ~FrameProcessor() = default;
		virtual cv::Mat processFrame(const cv::Mat& frame) = 0;
		virtual bool isProcessed() const = 0;
		virtual void resetState() = 0;
	};

	class CalibProcessor : public FrameProcessor {
	protected:
		cv::Ptr<calibrationData> mCalibData;
		TemplateType mBoardType;
		cv::Size mBoardSize;
		std::vector<cv::Point2f> mTemplateLocations;
		std::vector<cv::Point2f> mCurrentImagePoints;
		cv::Mat mCurrentCharucoCorners;
		cv::Mat mCurrentCharucoIds;

		cv::Ptr<cv::SimpleBlobDetector> mBlobDetectorPtr;

		cv::Ptr<cv::aruco::Dictionary> mArucoDictionary;
		cv::Ptr<cv::aruco::CharucoBoard> mCharucoBoard;


		int mNeededFramesNum;
		unsigned mDelayBetweenCaptures;
		int mCapuredFrames;
		double mMaxTemplateOffset;
		float mSquareSize;
		float mTemplDist;

		bool detectAndParseChessboard(const cv::Mat& frame);
		bool detectAndParseChAruco(const cv::Mat& frame);
		bool detectAndParseACircles(const cv::Mat& frame);
		bool detectAndParseDualACircles(const cv::Mat& frame);
		void saveFrameData();
		void showCaptureMessage(const cv::Mat &frame, const std::string& message);
		bool checkLastFrame();

	public:
		CalibProcessor(cv::Ptr<calibrationData> data, captureParameters& capParams);
		cv::Mat processFrame(const cv::Mat& frame) CV_OVERRIDE;
		bool isProcessed() const CV_OVERRIDE { return mCapuredFrames >= mNeededFramesNum; }
		void resetState() CV_OVERRIDE;
		~CalibProcessor() CV_OVERRIDE = default;
	};

	enum visualisationMode { Grid, Window };

	class ShowProcessor : public FrameProcessor {
	protected:
		cv::Ptr<calibrationData> mCalibdata;
		cv::Ptr<calibController> mController;
		TemplateType mBoardType;
		visualisationMode mVisMode;
		bool mNeedUndistort;
		double mGridViewScale;
		double mTextSize;

		void drawBoard(cv::Mat& img, cv::InputArray points);
		void drawGridPoints(const cv::Mat& frame);
	public:
		void switchVisualizationMode();
		void updateBoardsView();
		ShowProcessor(cv::Ptr<calibrationData> data, cv::Ptr<calibController> controller, TemplateType board);
		cv::Mat processFrame(const cv::Mat& frame) CV_OVERRIDE;
		bool isProcessed() const CV_OVERRIDE { return false; }
		void resetState() CV_OVERRIDE {}

		void setVisualizationMode(visualisationMode mode) { mVisMode = mode; }
		
		void clearBoardsView() { imshow(gridWindowName, cv::Mat()); }

		void switchUndistort() { mNeedUndistort = !mNeedUndistort; }
		void setUndistort(bool isEnabled) { mNeedUndistort = isEnabled; }

		~ShowProcessor() CV_OVERRIDE = default;
	};
}
#endif