#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "XeryonXDController.h"
#include "XeryonXDAxis.h"

// These are the XDAxis methods

/** Creates a new XDAxis object.
  * \param[in] pC Pointer to the ACRController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
XDAxis::XDAxis(XDController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;

  asynPrint(pC->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDAxis::XDAxis: Creating axis %u\n", axisNo);
  // channel_ = axisNo;
  // // Set hold time
  // sprintf(pC_->outString_, ":CHAN%d:HOLD %d", channel_, HOLD_FOREVER);
  // status = pC_->writeController();
  // pC_->clearErrors();
  // callParamCallbacks();
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
  if (level > 0) {
	int pcode;
	char pname[256];
	int channelState;
	int vel;
	int acc;
	int mclf;
    int followError;
	int error;
	int temp;

	asynStatus status;

	// sprintf(pC_->outString_, ":CHAN%d:PTYP?", channel_);
	// status = pC_->writeReadController();
	// pcode = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:PTYP:NAME?", channel_);
	// status = pC_->writeReadController();
	// strcpy(pC_->inString_, pname);
	// sprintf(pC_->outString_, ":CHAN%d:STAT?", channel_);
	// status = pC_->writeReadController();
	// channelState = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:VEL?", channel_);
	// status = pC_->writeReadController();
	// vel = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:ACC?", channel_);
	// status = pC_->writeReadController();
	// acc = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:MCLF?", channel_);
	// status = pC_->writeReadController();
	// mclf = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:FERR?", channel_);
	// status = pC_->writeReadController();
	// followError = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:ERR?", channel_);
	// status = pC_->writeReadController();
	// error = atoi(pC_->inString_);
	// sprintf(pC_->outString_, ":CHAN%d:TEMP?", channel_);
	// status = pC_->writeReadController();
	// temp = atoi(pC_->inString_);



  //   fprintf(fp, "  axis %d\n"
	// 		    " positioner type %d\n"
	// 			" positioner name %s\n"
	// 			" state %d\n"
	// 			" velocity %d\n"
	// 			" acceleration %d\n"
	// 			" max closed loop frequency %d\n"
	// 			" following error %d\n"
	// 			" error %d\n"
	// 			" temp %d\n",
  //           axisNo_, pcode, pname, channelState, vel,
	// 		acc, mclf, followError, error, temp);
	// pC_->clearErrors();
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


asynStatus XDAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "move";

  // /* XD move mode is:
  //  *	- absolute=0
  //  *	- relative=1
  //  */

  // // Set hold time
  // sprintf(pC_->outString_, ":CHAN%d:MMOD %d", channel_, relative>0?1:0);
  // status = pC_->writeController();
  // // Set acceleration
  // sprintf(pC_->outString_, ":CHAN%d:ACC %f", channel_, acceleration*PULSES_PER_STEP);
  // status = pC_->writeController();
  // // Set velocity
  // sprintf(pC_->outString_, ":CHAN%d:VEL %f", channel_, maxVelocity*PULSES_PER_STEP);
  // status = pC_->writeController();
  // // Do move
  // sprintf(pC_->outString_, ":MOVE%d %f", channel_, position*PULSES_PER_STEP);
  // status = pC_->writeController();

  return status;
}

asynStatus XDAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status=asynSuccess;
   static const char *functionName = "homeAxis";
  printf("Home command received %d\n", forwards);
  // unsigned short refOpt = 0;

  // if (forwards==0){
	//   refOpt |= START_DIRECTION;
  // }
  // refOpt |= AUTO_ZERO;

  // // Set default reference options - direction and autozero
  // printf("ref opt: %d\n", refOpt);
  // sprintf(pC_->outString_, ":CHAN%d:REF:OPT %d", channel_, refOpt);
  // status = pC_->writeController();
  // pC_->clearErrors();

  // // Set hold time
  // sprintf(pC_->outString_, ":CHAN%d:HOLD %d", channel_, HOLD_FOREVER);
  // status = pC_->writeController();
  // pC_->clearErrors();
  // // Set acceleration
  // sprintf(pC_->outString_, ":CHAN%d:ACC %f", channel_, acceleration*PULSES_PER_STEP);
  // status = pC_->writeController();
  // pC_->clearErrors();
  // // Set velocity
  // sprintf(pC_->outString_, ":CHAN%d:VEL %f", channel_, maxVelocity*PULSES_PER_STEP);
  // status = pC_->writeController();
  // pC_->clearErrors();
  // Begin move
  sprintf(pC_->outString_, "INDX=%d", forwards);
  status = pC_->writeController();
  // pC_->clearErrors();

  return status;
}

asynStatus XDAxis::stop(double acceleration )
{
  asynStatus status;
  static const char *functionName = "stopAxis";

  sprintf(pC_->outString_, "STOP=1");
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
  * the drive power-on status and positioner type. It does not current detect following error, etc.
  * but this could be added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus XDAxis::poll(bool *moving)
{ 
  int done;
  int chanState;
  int closedLoop;
  int calibrating;
  int referencing;
  int moveDelayed;
  int sensorPresent;
  int isCalibrated;
  int isReferenced;
  int endStopReached;
  int rangeLimitReached;
  int followLimitReached;
  int movementFailed;
  int isStreaming;
  int overTemp;
  int refMark;
  int positionerType;
  double encoderPosition;
  double theoryPosition;
  int driveOn;
  asynStatus comStatus = asynSuccess;

  // Read the channel state
  // sprintf(pC_->outString_, ":CHAN%d:STAT?", channel_);
  sprintf(pC_->outString_, "STAT=?");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // chanState = atoi(pC_->inString_);
  // done               = (chanState & ACTIVELY_MOVING)?0:1;
  // closedLoop         = (chanState & CLOSED_LOOP_ACTIVE)?1:0;
  // calibrating        = (chanState & CALIBRATING)?1:0;
  // referencing        = (chanState & REFERENCING)?1:0;
  // moveDelayed        = (chanState & MOVE_DELAYED)?1:0;
  // sensorPresent      = (chanState & SENSOR_PRESENT)?1:0;
  // isCalibrated       = (chanState & IS_CALIBRATED)?1:0;
  // isReferenced       = (chanState & IS_REFERENCED)?1:0;
  // endStopReached     = (chanState & END_STOP_REACHED)?1:0;
  // rangeLimitReached  = (chanState & RANGE_LIMIT_REACHED)?1:0;
  // followLimitReached = (chanState & FOLLOWING_LIMIT_REACHED)?1:0;
  // movementFailed     = (chanState & MOVEMENT_FAILED)?1:0;
  // isStreaming        = (chanState & STREAMING)?1:0;
  // overTemp           = (chanState & OVERTEMP)?1:0;
  // refMark            = (chanState & REFERENCE_MARK)?1:0;

  // *moving = done ? false:true;
  // setIntegerParam(pC_->motorStatusDone_, done);
  // setIntegerParam(pC_->motorClosedLoop_, closedLoop);
  // setIntegerParam(pC_->motorStatusHasEncoder_, sensorPresent);
  // setIntegerParam(pC_->motorStatusGainSupport_, sensorPresent);
  // setIntegerParam(pC_->motorStatusHomed_, isReferenced);
  // setIntegerParam(pC_->motorStatusHighLimit_, endStopReached);
  // setIntegerParam(pC_->motorStatusLowLimit_, endStopReached);
  // setIntegerParam(pC_->motorStatusFollowingError_, followLimitReached);
  // setIntegerParam(pC_->motorStatusProblem_, movementFailed);
  // setIntegerParam(pC_->motorStatusAtHome_, refMark);

  // // Read the current encoder position
  // sprintf(pC_->outString_, ":CHAN%d:POS?", channel_);
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // encoderPosition = (double)strtod(pC_->inString_, NULL);
  // encoderPosition /= PULSES_PER_STEP;
  // setDoubleParam(pC_->motorEncoderPosition_, encoderPosition);

  // // Read the current theoretical position
  // sprintf(pC_->outString_, ":CHAN%d:POS:TARG?", channel_);
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // theoryPosition = (double)strtod(pC_->inString_, NULL);
  // theoryPosition /= PULSES_PER_STEP;
  // setDoubleParam(pC_->motorPosition_, theoryPosition);

  // // Read the drive power on status
  // sprintf(pC_->outString_, ":CHAN%d:AMPL?", channel_);
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // driveOn = atoi(pC_->inString_) ? 1:0;
  // setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  // setIntegerParam(pC_->motorStatusProblem_, 0);

  // // Read the currently selected positioner type
  // sprintf(pC_->outString_, ":CHAN%d:PTYP?", channel_);
  // comStatus = pC_->writeReadController();
  // if (comStatus) goto skip;
  // positionerType = atoi(pC_->inString_);
  // setIntegerParam(pC_->ptyprb_, positionerType);

  skip:
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDAxis: poll skipped: %s\n", pC_->inString_);
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}