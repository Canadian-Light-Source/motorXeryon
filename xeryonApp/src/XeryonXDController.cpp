#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "XeryonXDController.h"
#include "XeryonXDAxis.h"

static const char *driverName = "XeryonXDMotorDriver";

/** Creates a new XDController object.
 * \param[in] portName             The name of the asyn port that will be created for this driver
 * \param[in] XDPortName         The name of the drvAsynIPPPort that was created previously to connect to the XD controller
 * \param[in] numAxes              The number of axes that this controller supports
 * \param[in] movingPollPeriod     The time between polls when any axis is moving
 * \param[in] idlePollPeriod       The time between polls when no axis is moving
 */
XDController::XDController(const char *portName, const char *XDPortName, int numAxes,
                           double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_XD_PARAMS,
                          0, 0,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    int axis;
    asynStatus status;
    static const char *functionName = "XDController";
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Creating controller\n");

    // Create controller-specific parameters
    // createParam(XDMclfString, asynParamInt32, &this->mclf_);
    // createParam(XDPtypString, asynParamInt32, &this->ptyp_);
    // createParam(XDPtypRbString, asynParamInt32, &this->ptyprb_);
    // createParam(XDCalString, asynParamInt32, &this->cal_);

    /* Connect to XD controller */
    status = pasynOctetSyncIO->connect(XDPortName, 0, &pasynUserController_, NULL);
    pasynOctetSyncIO->setInputEos(pasynUserController_, "\n", 1);
    pasynOctetSyncIO->setOutputEos(pasynUserController_, "\n", 1);

    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Connecting to controller\n");
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: cannot connect to XD controller\n",
                  driverName, functionName, status);
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Clearing error messages\n");
    // this->clearErrors();

    sprintf(this->outString_, "SOFT=?");
    status = this->writeReadController();
    if (status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: cannot connect obtain software version from controller\n",
                  driverName, functionName, status);
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDController: Software version: %s\n", this->inString_);
    // this->clearErrors();

    // Create the axis objects
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Creating axes\n");
    for (axis = 0; axis < numAxes; axis++)
    {
        new XDAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new XDController object.
 * Configuration command, called directly or from iocsh
 * \param[in] portName          The name of the asyn port that will be created for this driver
 * \param[in] XDPortName      The name of the drvAsynIPPPort that was created previously to connect to the XD controller
 * \param[in] numAxes           The number of axes that this controller supports
 * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
 * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
 */
extern "C" int XDCreateController(const char *portName, const char *XDPortName, int numAxes,
                                  int movingPollPeriod, int idlePollPeriod)
{
    new XDController(portName, XDPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    return (asynSuccess);
}

asynStatus XDController::poll()
{
  static const char *functionName = "XDController::poll";
  sprintf(this->outString_, "EPOS=?");
  asynStatus status = this->writeReadController();
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect obtain encoder position from controller\nStatus:%d",
      driverName, functionName, status);
  }
  asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDController: EPOS: %s\n", this->inString_);
  // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDController:poll;\n");
}

/** Reports on status of the driver
 * \param[in] fp The file pointer on which report information will be written
 * \param[in] level The level of report detail desired
 *
 * If details > 0 then information is printed about each axis.
 * After printing controller-specific information calls asynMotorController::report()
 */
void XDController::report(FILE *fp, int level)
{
    fprintf(fp, "XD motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
            this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

/** Returns a pointer to an XDMotorAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] pasynUser asynUser structure that encodes the axis index number. */
XDAxis *XDController::getAxis(asynUser *pasynUser)
{
    return static_cast<XDAxis *>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an XDMotorAxis object.
 * Returns NULL if the axis number encoded in pasynUser is invalid.
 * \param[in] axisNo Axis index number. */
XDAxis *XDController::getAxis(int axisNo)
{
    return static_cast<XDAxis *>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
 * Extracts the function and axis number from pasynUser.
 * Sets the value in the parameter library.
 * For all other functions it calls asynMotorController::writeInt32.
 * Calls any registered callbacks for this pasynUser->reason and address.
 * \param[in] pasynUser asynUser structure that encodes the reason and address.
 * \param[in] value     Value to write. */
asynStatus XDController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    // XDAxis *pAxis = getAxis(pasynUser);
    // static const char *functionName = "writeInt32";

    // /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
    //  * status at the end, but that's OK */
    // status = setIntegerParam(pAxis->axisNo_, function, value);

    // if (function == mclf_) {
    //   /* set MCLF */
    //   sprintf(pAxis->pC_->outString_, ":CHAN%d:MCLF:CURR %d", pAxis->axisNo_, value);
    //   status = pAxis->pC_->writeController();
    // }
    // else if (function == ptyp_) {
    //   /* set positioner type */
    //   sprintf(pAxis->pC_->outString_, ":CHAN%d:PTYP %d", pAxis->axisNo_, value);
    //   status = pAxis->pC_->writeController();
    // }
    // else if (function == cal_) {
    //   /* send calibration command */
    //   sprintf(pAxis->pC_->outString_, ":CAL%d", pAxis->axisNo_);
    //   status = pAxis->pC_->writeController();
    // }
    // else {
    //   /* Call base class method */
    //   status = asynMotorController::writeInt32(pasynUser, value);
    // }

    // /* Do callbacks so higher layers see any changes */
    // callParamCallbacks(pAxis->axisNo_);
    // if (status)
    //   asynPrint(pasynUser, ASYN_TRACE_ERROR,
    //       "%s:%s: error, status=%d function=%d, value=%d\n",
    //       driverName, functionName, status, function, value);
    // else
    //   asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
    //       "%s:%s: function=%d, value=%d\n",
    //       driverName, functionName, function, value);
    return status;
}

/** Code for iocsh registration */
static const iocshArg XDCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg XDCreateControllerArg1 = {"XD port name", iocshArgString};
static const iocshArg XDCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg XDCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg XDCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const XDCreateControllerArgs[] = {&XDCreateControllerArg0,
                                                            &XDCreateControllerArg1,
                                                            &XDCreateControllerArg2,
                                                            &XDCreateControllerArg3,
                                                            &XDCreateControllerArg4};
static const iocshFuncDef XDCreateControllerDef = {"XDCreateController", 5, XDCreateControllerArgs};
static void XDCreateContollerCallFunc(const iocshArgBuf *args)
{
  XDCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void XDMotorRegister(void)
{
  iocshRegister(&XDCreateControllerDef, XDCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(XDMotorRegister);
}