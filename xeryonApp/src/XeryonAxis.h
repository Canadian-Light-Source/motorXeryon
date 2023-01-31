#ifndef XERYON_AXIS_H
#define XERYON_AXIS_H

#include <array>
#include <string>
#include <iostream>

#include "XeryonStages.h"

class XeryonAxis
{
public:
    XeryonAxis(){};

    /**
     * @brief Print report of stage parameters to std::cout.
     */
    void stageReport()
    {
        std::cout << "Xeryon stage report:" << std::endl;
        std::cout << "isLinear:       " << stage->isLinear << std::endl;
        std::cout << "encoderRes:     " << stage->encoderRes << std::endl;
        std::cout << "encoderResCmd:  " << stage->encoderResCmd << std::endl;
        std::cout << "velocityFactor: " << stage->velocityFactor << std::endl;
    }

    /**
     * @brief Get the stage resolution.
     * @return resolution in nm/step or deg/step
     */
    double getResolution() { return stage->encoderRes; };

    /**
     * @brief Get the velocity factor.
     * @return velocity factor
     */
    double getVelocityFactor() { return stage->velocityFactor; };

    /**
     * @brief Get the encoder resolution command.
     * @return command for encoder resolution setting
     */
    std::string getEncoderResCmd() { return stage->encoderResCmd; };

    /**
     * @brief Linear or angular axis.
     * @return true for linear axis, false for angular axis
     */
    bool isLinear() { return stage->isLinear; };

    /**
     * @brief Set the status word.
     * @param[in] s status word
     */
    void setStatus(int s)
    {
        status = s;
        isAmpEnabled = (status & (1 << 1));
        isForceZero = (status & (1 << 4));
        isMotorOn = (status & (1 << 5));
        isClosedLoop = (status & (1 << 6));
        isEncoderAtIndex = (status & (1 << 7));
        isEncoderValid = (status & (1 << 8));
        isSearchingIndex = (status & (1 << 9));
        isPositionReached = (status & (1 << 10));
        isEncoderError = (status & (1 << 12));
        isScanning = (status & (1 << 13));
        isAtLeftEnd = (status & (1 << 14));
        isAtRightEnd = (status & (1 << 15));
        isErrorLimit = (status & (1 << 16));
        isSearchingOptimalFrequency = (status & (1 << 17));
    };

    /**
     * @brief get the status word.
     * @return status word
     */
    int getStatus() { return status; };

    /**
     * @brief Set the stage type.
     * @param[in] type stage type
     */
    void setStage(std::string type) { stage = stages.getStage(type); };

    /**
     * @brief Gget the indivual status bits.
     */
    int getIsAmpEnabled() { return isAmpEnabled; }
    int getIsForceZero() { return isForceZero; }
    int getIsMotorOn() { return isMotorOn; }
    int getIsClosedLoop() { return isClosedLoop; }
    int getIsEncoderAtIndex() { return isEncoderAtIndex; }
    int getIsEncoderValid() { return isEncoderValid; }
    int getIsSearchingIndex() { return isSearchingIndex; }
    int getIsPositionReached() { return isPositionReached; }
    int getIsEncoderError() { return isEncoderError; }
    int getIsScanning() { return isScanning; }
    int getIsAtLeftEnd() { return isAtLeftEnd; }
    int getIsAtRightEnd() { return isAtRightEnd; }
    int getIsErrorLimit() { return isErrorLimit; }
    int getIsSearchingOptimalFrequency() { return isSearchingOptimalFrequency; }

    std::shared_ptr<XeryonStage> stage = std::make_shared<XeryonStage>(false, "", 1, 1); // default initialization

private:
    uint status;
    bool isAmpEnabled;
    bool isForceZero;
    bool isMotorOn;
    bool isClosedLoop;
    bool isEncoderAtIndex;
    bool isEncoderValid;
    bool isSearchingIndex;
    bool isPositionReached;
    bool isEncoderError;
    bool isScanning;
    bool isAtLeftEnd;
    bool isAtRightEnd;
    bool isErrorLimit;
    bool isSearchingOptimalFrequency;

    XeryonStages stages = XeryonStages();
};

#endif /* XERYON_AXIS_H */
