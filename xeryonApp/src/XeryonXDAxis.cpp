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

XDAxis::XDAxis(XDController *pC, int axisNo)
    : XeryonAxis(), asynMotorAxis(pC, axisNo),
      pC_(pC)
{
  // asynStatus status;

  asynPrint(pC->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::XDAxis: Creating axis %u\n", axisNo);
  // stop unsolicited data transfer
  // pC_->sendCommand(this->pC_, axisNo, "INFO", "0");
  pC_->setParameter(this->pC_, axisNo, "INFO", "0");
  // sprintf(pC_->outString_, "INFO=0");
  // status = pC_->writeController();
  // if (status)
  // {
  //   asynPrint(pC->pasynUserSelf, ASYN_TRACE_ERROR,
  //             "cannot connect to XD controller\n");
  // }

  callParamCallbacks();
}

void XDAxis::report(FILE *fp, int level)
{
  if (level > 0)
  {
    fprintf(fp, "Xeryon axis\n");
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus XDAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{

  asynStatus status = asynSuccess;

  sprintf(pC_->outString_, "SSPD=%d", (int)(maxVelocity * this->getResolution() * this->getVelocityFactor()));
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

  // Begin move
  sprintf(pC_->outString_, "INDX=%d", forwards);
  status = pC_->writeController();

  return status;
}

asynStatus XDAxis::stop(double acceleration)
{
  asynStatus status;

  // Force the piezo signals to zero volt
  sprintf(pC_->outString_, "ZERO");
  status = pC_->writeController();

  return status;
}

asynStatus XDAxis::poll(bool *moving)
{

  /*
    TODO:
      - get rid of the ugly `goto`
  */

  int chanState;

  int reply = 0;
  std::string replyString;

  double encoderPosition;
  double targetPosition;
  double targetVelocity;
  asynStatus comStatus = asynSuccess;

  // Read the channel state
  // sprintf(pC_->outString_, "STAT=?");
  // comStatus = pC_->writeReadController();
  // asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDAxis: send(%s)->(%s) [%d]\n", pC_->outString_, pC_->inString_, comStatus);
  // if (comStatus)
  //   goto skip;

  // chanState = (int)this->decodeReply(pC_->inString_);
  // pC_->getParameter(this->pC_, "STAT", replyString);
  // chanState = atoi(replyString.c_str());
  pC_->getParameter(this->pC_, "STAT", chanState);
  // (int)this->decodeReply(pC_->inString_);
  setIntegerParam(pC_->statrb_, chanState);
  this->setStatus(chanState);

  *moving = !this->getIsPositionReached();
  setIntegerParam(pC_->motorStatusDone_, ((this->getIsPositionReached()) || (this->getIsForceZero())));
  setIntegerParam(pC_->motorClosedLoop_, this->getIsClosedLoop());
  setIntegerParam(pC_->motorStatusHasEncoder_, 1); // Xeryon axis have encoders
  setIntegerParam(pC_->motorStatusGainSupport_, !this->getIsForceZero());
  setIntegerParam(pC_->motorStatusHomed_, this->getIsEncoderValid());
  setIntegerParam(pC_->motorStatusHighLimit_, this->getIsAtLeftEnd());
  setIntegerParam(pC_->motorStatusLowLimit_, this->getIsAtRightEnd());
  setIntegerParam(pC_->motorStatusFollowingError_, this->getIsErrorLimit());
  setIntegerParam(pC_->motorStatusProblem_, this->getIsErrorLimit());
  setIntegerParam(pC_->motorStatusAtHome_, this->getIsEncoderAtIndex());

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

skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}
