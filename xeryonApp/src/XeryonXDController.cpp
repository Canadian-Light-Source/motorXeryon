#include <iocsh.h>
#include <epicsThread.h>
#include <epicsExit.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "XeryonXDController.h"
#include "XeryonXDAxis.h"

static const char *driverName = "XeryonXDMotorDriver";

XDController::XDController(const char *portName, const char *XDPortName, int numAxes,
                           double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_XD_PARAMS,
                          0, 0,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    static const char *functionName = "XDController";
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Creating controller\n");

    // Create controller-specific parameters
    createParam(XDstatString, asynParamInt32, &this->statrb_); // whole positioner status word
    createParam(XDsspdString, asynParamInt32, &this->sspdrb_); // sspd readback
    createParam(XDeposString, asynParamInt32, &this->eposrb_); // epos readback
    createParam(XDdposString, asynParamInt32, &this->dposrb_); // dpos readback
    // extra stage info
    createParam(XDfreqString, asynParamInt32, &this->freqrb_);

    // stage commands
    createParam(XDindxString, asynParamInt32, &this->indx_);
    createParam(XDptolString, asynParamInt32, &this->ptol_);
    createParam(XDpto2String, asynParamInt32, &this->pto2_);

    // LED test
    createParam(XDtestString, asynParamInt32, &this->test_);

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

    // Create the axis objects
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, "XDController::XDController: Creating axes\n");
    for (size_t axis = 0; axis < numAxes; axis++)
    {
        controllerAxes[axis] = std::make_shared<XDAxis>(this, axis);
    }

    int reply;
    getParameter(this, "SOFT", reply);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDController: software verions: %d\n", reply);
    getParameter(this, "SRNO", reply);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "XDController::XDController: serial number: %d\n", reply);
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/**
 * @brief Creates a new XDController object.
 * @details Configuration command, called directly or from iocsh
 * @param[in] portName The name of the asyn port that will be created for this driver
 * @param[in] XDPortName The name of the drvAsynIPPPort that was created previously to connect to the XD controller
 * @param[in] numAxes The number of axes that this controller supports
 * @param[in] movingPollPeriod The time in ms between polls when any axis is moving
 * @param[in] idlePollPeriod The time in ms between polls when no axis is moving
 */
int XDCreateController(const std::string &portName, const std::string &XDPortName, const uint16_t numAxes, const double movingPollPeriod, const double idlePollPeriod)
{
    try
    {
        ControllerHolder::getInstance().addController(portName, XDPortName, numAxes, movingPollPeriod, idlePollPeriod);
    }
    catch (const std::runtime_error &e)
    {
        std::cout << "Driver configuration problem: " << e.what() << std::endl
                  << "Aborting initialization..." << std::endl;
        epicsExit(-1);
    }
    return (asynSuccess);
}

/**
 * @brief Configures an axis object in a respective controller.
 * @details Configuration command, called directly or from iocsh
 * @param[in] portName The name of the asyn port that will be created for this driver
 * @param[in] axisNo The unique number of the axis object in the controller
 * @param[in] stageType The type of the stage that is connected to the controller
 */
int XDconfigureAxis(const std::string &portName, const int axisNo, const char *stageType)
{
    try
    {
        XDController *device =
            ControllerHolder::getInstance().getController(portName).get();
        std::shared_ptr<XDAxis> pAxis = device->getAxisPointer(axisNo);
        pAxis->setStage(stageType);
    }
    catch (const std::out_of_range &e)
    {
        std::cout << "Controller with provided name and function does not exist. "
                  << "Exception: " << e.what() << std::endl;
        return (asynError);
    }
    return (asynSuccess);
};

void XDController::report(FILE *fp, int level)
{
    fprintf(fp, "XD motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
            this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

XDAxis *XDController::getAxis(asynUser *pasynUser)
{
    return static_cast<XDAxis *>(asynMotorController::getAxis(pasynUser));
}

XDAxis *XDController::getAxis(int axisNo)
{
    return static_cast<XDAxis *>(asynMotorController::getAxis(axisNo));
}

asynStatus XDController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    XDAxis *pAxis = getAxis(pasynUser);
    static const char *functionName = "writeInt32";

    // /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
    //  * status at the end, but that's OK */
    status = setIntegerParam(pAxis->axisNo_, function, value);

    if (function == indx_)
    {
        /* move to index (homing) */
        sprintf(pAxis->pC_->outString_, "INDX=%d", value);
        status = pAxis->pC_->writeController();
    }
    else if (function == ptol_)
    {
        sprintf(pAxis->pC_->outString_, "PTOL=%d", value);
        status = pAxis->pC_->writeController();
    }
    else if (function == pto2_)
    {
        sprintf(pAxis->pC_->outString_, "PTO2=%d", value);
        status = pAxis->pC_->writeController();
    }
    else if (function == test_)
    {   
        pAxis->pC_->setParameter(pAxis->pC_, "TEST", value);
        // sprintf(pAxis->pC_->outString_, "TEST=%d", value);
        // status = pAxis->pC_->writeController();
    }
    else
    {
        /* Call base class method */
        status = asynMotorController::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(pAxis->axisNo_);
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s: error, status=%d function=%d, value=%d\n",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  driverName, functionName, function, value);
    return status;
}

void XDController::setParameter(XDController *device, const std::string &cmd, const int &payload)
{
    XDAxis *pAxis = getAxis(this->pasynUserSelf);
    std::cout << "--> AXIS: " << pAxis->axisNo_ << std::endl;
    sprintf(device->outString_, "%s=%d", cmd.c_str(), payload);
    asynStatus status = device->writeController();
    if (status)
    {
        throw XeryonControllerException("Failed to set parameter" + std::string(device->outString_));
    }
};

void XDController::getParameter(XDController *device, const std::string &cmd, int &reply)
{
    sprintf(device->outString_, "%s=?", cmd.c_str());
    asynStatus status = device->writeReadController();
    if (status)
    {
        throw XeryonControllerException("Failed to get parameter" + std::string(device->outString_));
    }
    std::string buf = device->inString_;
    try
    {
        reply = atoi(buf.substr(buf.find("=") + 1).c_str());
    }
    catch (const std::exception &e)
    {
        std::cerr << "failed to decode controller reply (" << buf << "). Exception: " << e.what() << '\n';
    }

};

void ControllerHolder::addController(const std::string &portName, const std::string &XDPortName, const uint16_t numAxes, const double movingPollPeriod, const double idlePollPeriod)
{
    if (controllerMap.find(portName) == controllerMap.end())
    {
        std::pair<std::string, std::shared_ptr<XDController>> controller(portName, std::shared_ptr<XDController>(new XDController(portName.c_str(), XDPortName.c_str(), numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.)));
        controllerMap.insert(controller);
    }
    else
    {
        // Display warning.
        std::cout << "Controller with the name " << portName << " already exists. Skipping configuration." << std::endl;
    }
}

std::shared_ptr<XDController> ControllerHolder::getController(const std::string &portName)
{
    return controllerMap.at(portName);
}

/** Code for iocsh registration */
static const iocshArg XDCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg XDCreateControllerArg1 = {"XD port name", iocshArgString};
static const iocshArg XDCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg XDCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg XDCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const XDCreateControllerArgs[] = {&XDCreateControllerArg0,
                                                         &XDCreateControllerArg1,
                                                         &XDCreateControllerArg2,
                                                         &XDCreateControllerArg3,
                                                         &XDCreateControllerArg4};
static const iocshFuncDef XDCreateControllerDef = {"XDCreateController", 5, XDCreateControllerArgs};
static void XDCreateContollerCallFunc(const iocshArgBuf *args)
{
    XDCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg XDconfigureAxisArg0 = {"Port name", iocshArgString};
static const iocshArg XDconfigureAxisArg1 = {"Axis Number", iocshArgInt};
static const iocshArg XDconfigureAxisArg2 = {"type of stage", iocshArgString};
static const iocshArg *const XDconfigureAxisArgs[] = {&XDconfigureAxisArg0,
                                                      &XDconfigureAxisArg1,
                                                      &XDconfigureAxisArg2};
static const iocshFuncDef XDconfigureAxisDef = {"XDconfigureAxis", 3, XDconfigureAxisArgs};
static void XDconfigureAxisCallFunc(const iocshArgBuf *args)
{
    XDconfigureAxis(args[0].sval, args[1].ival, args[2].sval);
}
static void XDMotorRegister(void)
{
    iocshRegister(&XDCreateControllerDef, XDCreateContollerCallFunc);
    iocshRegister(&XDconfigureAxisDef, XDconfigureAxisCallFunc);
}

extern "C"
{
    epicsExportRegistrar(XDMotorRegister);
}