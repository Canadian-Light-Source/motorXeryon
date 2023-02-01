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
  asynPrint(pC->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::XDAxis: Creating axis %u\n", axisNo);

  try
  {
    // stop unsolicited data transfer
    pC_->setParameter(this->pC_, "INFO", 0);
    callParamCallbacks();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    asynPrint(this->pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::XDAxis: %s\n", e.what());
  }
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

  try
  {
    int velocity = (int)(maxVelocity * this->getResolution() * this->getVelocityFactor());
    pC_->setParameter(this->pC_, "SSPD", velocity);

    // set absolute or relative movement target
    if (relative)
    {
      pC_->setParameter(this->pC_, "STEP", (int)position);
    }
    else
    {
      pC_->setParameter(this->pC_, "DPOS", (int)position);
    }
  }
  catch (const std::exception &e)
  {
    asynPrint(this->pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::move: %s\n", e.what());
    status = asynError;
  }

  return status;
}

asynStatus XDAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;

  try
  {
    // Begin move
    pC_->setParameter(this->pC_, "INDX", forwards);
  }
  catch (const std::exception &e)
  {
    asynPrint(this->pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::home: %s\n", e.what());
    status = asynError;
  }

  return status;
}

asynStatus XDAxis::stop(double acceleration)
{
  asynStatus status = asynSuccess;

  try
  {
    // Force the piezo signals to zero volt
    pC_->setParameter(this->pC_, "ZERO");
  }
  catch (const std::exception &e)
  {
    asynPrint(this->pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDAxis::stop: %s\n", e.what());
    status = asynError;
  }

  return status;
}

asynStatus XDAxis::poll(bool *moving)
{
  int reply = 0;
  asynStatus comStatus = asynSuccess;

  try
  {
    // Read the channel state
    pC_->getParameter(this->pC_, "STAT", reply);
    setIntegerParam(pC_->statrb_, reply);
    this->setStatus(reply);

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

    // Read the encoder position
    pC_->getParameter(this->pC_, "EPOS", reply);
    setDoubleParam(pC_->motorEncoderPosition_, (double)reply);
    setIntegerParam(pC_->eposrb_, reply);

    // Read the current theoretical position
    pC_->getParameter(this->pC_, "DPOS", reply);
    setDoubleParam(pC_->motorPosition_, reply);
    setIntegerParam(pC_->dposrb_, reply);

    // Read the current velocity setpoint
    pC_->getParameter(this->pC_, "SSPD", reply);
    setIntegerParam(pC_->sspdrb_, reply);

    // Read the current exitation frequency
    pC_->getParameter(this->pC_, "FREQ", reply);
    setIntegerParam(pC_->freqrb_, reply);
  }
  catch (const std::exception &e)
  {
    std::cerr << "error in poll" << e.what() << '\n';
    comStatus = asynError;
  }
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1 : 0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}
