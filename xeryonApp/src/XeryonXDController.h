#ifndef XERYON_XD_CONROLLER_H
#define XERYON_XD_CONROLLER_H

#include "asynMotorController.h"
#include "XeryonXDAxis.h"

class epicsShareClass XDController : public asynMotorController
{
public:
    XDController(const char *portName, const char *XDPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
    // virtual asynStatus clearErrors();

    /* These are the methods that we override from asynMotorDriver */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus poll();

    /* These are the methods that we override from asynMotorDriver */
    void report(FILE *fp, int level);
    XDAxis *getAxis(asynUser *pasynUser);
    XDAxis *getAxis(int axisNo);

// from smarAct still needs fixing
// TODO: check and add support for XD controller features
protected:
    int mclf_; /**< MCL frequency */
#define FIRST_XD_PARAM mclf_
    int ptyp_;   /**< positioner type */
    int ptyprb_; /**< positioner type readback */
    int cal_;    /**< calibration command */
#define LAST_XD_PARAM cal_
#define NUM_XD_PARAMS (&LAST_XD_PARAM - &FIRST_XD_PARAM + 1)

    friend class XDAxis;
};

#endif /* XERYON_XD_CONROLLER_H */