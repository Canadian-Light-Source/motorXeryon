#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "XeryonXDController.h"
#include "XeryonXDAxis.h"
#include "XeryonAxis.h"

// These are the XDAxis methods

/** Creates a new XDAxis object.
 * \param[in] pC Pointer to the XDController to which this axis belongs.
 * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
 *
 * Initializes register numbers, etc.
 */
XDAxis::XDAxis(XDController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  asynStatus status;

  asynPrint(pC->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::XDAxis: Creating axis %u\n", axisNo);
  channel_ = axisNo;
  // stop unsolicited data transfer
  sprintf(pC_->outString_, "INFO=0");
  status = pC_->writeController();

  callParamCallbacks();
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information calls asynMotorController::report()
 */
void XDAxis::report(FILE *fp, int level)
{
  if (level > 0)
  {
    asynStatus status;

    //   fprintf(fp, " foo %d ", status);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus XDAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "move";

  // Set velocity
  /*
    TODO: SSPD is in weird units, 1 um/s for linear actuators and 0.01 deg/s for angular
    need to find a way to obtain MRES in here to scale the velocity properly
  */
  sprintf(pC_->outString_, "SSPD=%d", (int)maxVelocity);
  status = pC_->writeController();

  // set absolute or relative movement target
  if (relative)
  {
    sprintf(pC_->outString_, "STEP=%d", (int)position);
  }
  else
  {
    sprintf(pC_->outString_, "DPOS=%d", (int)position);
  }
  status = pC_->writeController();

  return status;
}

asynStatus XDAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "homeAxis";

  std::cout << "III: " << functionName << " home command: " << forwards << std::endl;

  // Begin move
  sprintf(pC_->outString_, "INDX=%d", forwards);
  status = pC_->writeController();

  return status;
}

asynStatus XDAxis::stop(double acceleration)
{
  asynStatus status;
  static const char *functionName = "stopAxis";

  // Force the piezo signals to zero volt
  sprintf(pC_->outString_, "ZERO");
  status = pC_->writeController();

  return status;
}

// asynStatus XDAxis::setPosition(double position)
// {
//   asynStatus status=asynSuccess;

//   // printf("Set position receieved\n");
//   // sprintf(pC_->outString_, ":CHAN%d:POS %f", channel_, position*PULSES_PER_STEP);
//   // status = pC_->writeController();
//   return status;
// }

/** Polls the axis.
 * This function reads the controller position, encoder position, the limit status, the moving status,
 * the drive power-on status and positioner type.
 * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
 * and then calls callParamCallbacks() at the end.
 * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus XDAxis::poll(bool *moving)
{

  /*
    TODO:
      - get rid of the ugly `goto`
      - out source status to a utility class instance to clean up the code here
  */

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

  int chanState;

  int reply=0;

  double encoderPosition;
  double targetPosition;
  double targetVelocity;
  asynStatus comStatus = asynSuccess;

  // Read the channel state
  sprintf(pC_->outString_, "STAT=?");
  comStatus = pC_->writeReadController();
  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  if (comStatus)
    goto skip;

  chanState = (int)this->decodeReply(pC_->inString_);
  setIntegerParam(pC_->statrb_, chanState);

  isAmpEnabled = (chanState & (1 << 1));
  isForceZero = (chanState & (1 << 4));
  isMotorOn = (chanState & (1 << 5));
  isClosedLoop = (chanState & (1 << 6));
  isEncoderAtIndex = (chanState & (1 << 7));
  isEncoderValid = (chanState & (1 << 8));
  isSearchingIndex = (chanState & (1 << 9));
  isPositionReached = (chanState & (1 << 10));
  isEncoderError = (chanState & (1 << 12));
  isScanning = (chanState & (1 << 13));
  isAtLeftEnd = (chanState & (1 << 14));
  isAtRightEnd = (chanState & (1 << 15));
  isErrorLimit = (chanState & (1 << 16));
  isSearchingOptimalFrequency = (chanState & (1 << 17));

  *moving = !isPositionReached;
  setIntegerParam(pC_->motorStatusDone_, (isPositionReached || isForceZero));
  setIntegerParam(pC_->motorClosedLoop_, isClosedLoop);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1); // Xeryon axis have encoders
  setIntegerParam(pC_->motorStatusGainSupport_, !isForceZero);
  setIntegerParam(pC_->motorStatusHomed_, isEncoderValid);
  setIntegerParam(pC_->motorStatusHighLimit_, isAtLeftEnd);
  setIntegerParam(pC_->motorStatusLowLimit_, isAtRightEnd);
  setIntegerParam(pC_->motorStatusFollowingError_, isErrorLimit);
  setIntegerParam(pC_->motorStatusProblem_, isErrorLimit);
  setIntegerParam(pC_->motorStatusAtHome_, isEncoderAtIndex);

  // Read the current encoder position
  sprintf(pC_->outString_, "EPOS=?");
  comStatus = pC_->writeReadController();
  // std::cout << "EPOS was decoded as --> " << this->decodeReply(pC_->inString_) << std::endl;
  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  if (comStatus)
    goto skip;
  // encoderPosition = (double)strtod(pC_->inString_, NULL);
  // encoderPosition /= PULSES_PER_STEP;
  encoderPosition = this->decodeReply(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_, (double)encoderPosition);
  setIntegerParam(pC_->eposrb_, encoderPosition);

  // Read the current theoretical position
  sprintf(pC_->outString_, "DPOS=?");
  comStatus = pC_->writeReadController();
  // std::cout << "DPOS was decoded as --> " << this->decodeReply(pC_->inString_) << std::endl;
  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  if (comStatus)
    goto skip;
  // targetPosition = (double)strtod(pC_->inString_, NULL);
  // targetPosition /= PULSES_PER_STEP;
  targetPosition = this->decodeReply(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, targetPosition);
  setIntegerParam(pC_->dposrb_, targetPosition);

  // Read the current velocity setpoint
  sprintf(pC_->outString_, "SSPD=?");
  comStatus = pC_->writeReadController();
  asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  if (comStatus)
    goto skip;
  // targetPosition = (double)strtod(pC_->inString_, NULL);
  // targetPosition /= PULSES_PER_STEP;
  targetVelocity = this->decodeReply(pC_->inString_);
  setIntegerParam(pC_->sspdrb_, targetVelocity);

  // Read the current exitation frequency
  sprintf(pC_->outString_, "FREQ=?");
  comStatus = pC_->writeReadController();
  // asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  if (comStatus)
    goto skip;
  reply = this->decodeReply(pC_->inString_);
  setIntegerParam(pC_->freqrb_, reply);

  // // Read the drive power on status
  // sprintf(pC_->outString_, "ENBL=?");
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // // driveOn = atoi(pC_->inString_) ? 1:0;
  // // setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  // setIntegerParam(pC_->motorStatusProblem_, 1);

  // // Read the currently selected positioner type
  // sprintf(pC_->outString_, ":CHAN%d:PTYP?", channel_);
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // positionerType = atoi(pC_->inString_);
  // setIntegerParam(pC_->ptyprb_, positionerType);

skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}
