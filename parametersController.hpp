#ifndef PARAMETERS_CONTROLLER_HPP
#define PARAMETERS_CONTROLLER_HPP

#include <string>
#include <opencv2/core.hpp>
#include "calibCommon.hpp"

namespace calib {

class parametersController
{
protected:
    captureParameters mCapParams;
    internalParameters mInternalParameters;

    bool loadFromFile(const std::string& inputFileName);
public:
	parametersController() = default;

	captureParameters getCaptureParameters() const { return mCapParams; }
	internalParameters getInternalParameters() const { return mInternalParameters; }

    bool loadFromParser(cv::CommandLineParser& parser);
};

}

#endif
