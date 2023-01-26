#ifndef XERYON_XD_CONROLLER_H
#define XERYON_XD_CONROLLER_H

#include "asynMotorController.h"
#include "XeryonXDAxis.h"

/** MCS2 Axis status flags **/
// TODO: XD mapping
const unsigned short ACTIVELY_MOVING = 0x0001;
const unsigned short CLOSED_LOOP_ACTIVE = 0x0002;
const unsigned short CALIBRATING = 0x0004;
const unsigned short REFERENCING = 0x0008;
const unsigned short MOVE_DELAYED = 0x0010;
const unsigned short SENSOR_PRESENT = 0x0020;
const unsigned short IS_CALIBRATED = 0x0040;
const unsigned short IS_REFERENCED = 0x0080;
const unsigned short END_STOP_REACHED = 0x0100;
const unsigned short RANGE_LIMIT_REACHED = 0x0200;
const unsigned short FOLLOWING_LIMIT_REACHED = 0x0400;
const unsigned short MOVEMENT_FAILED = 0x0800;
const unsigned short STREAMING = 0x1000;
const unsigned short OVERTEMP = 0x4000;
const unsigned short REFERENCE_MARK = 0x8000;

#define XDstatString "STAT"
#define XDsspdString "SSPD"
#define XDeposString "EPOS"
#define XDdposString "DPOS"

class epicsShareClass XDController : public asynMotorController
{
public:
    XDController(const char *portName, const char *XDPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
    // virtual asynStatus clearErrors();

    /* These are the methods that we override from asynMotorDriver */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    // asynStatus poll();

    /* These are the methods that we override from asynMotorDriver */
    void report(FILE *fp, int level);
    XDAxis *getAxis(asynUser *pasynUser);
    XDAxis *getAxis(int axisNo);

private:
    std::array<std::string, 3> XD_M_Axes{{"X", "Y", "Z"}};
    std::array<std::string, 12> XD_19_Axes{{"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"}};
    // from smarAct still needs fixing
    // TODO: check and add support for XD controller features
protected:
    int mclf_; /**< MCL frequency */
#define FIRST_XD_PARAM mclf_
    int statrb_; /**< axis status word readback */
    int eposrb_; /**< axis encoder readback */
    int dposrb_; /**< axis target position readback */
    int sspdrb_; /**< axis velocity setpoiny readback */
    int ptyp_;   /**< positioner type */
    int ptyprb_; /**< positioner type readback */
    int cal_;    /**< calibration command */
#define LAST_XD_PARAM cal_
#define NUM_XD_PARAMS (&LAST_XD_PARAM - &FIRST_XD_PARAM + 1)

    friend class XDAxis;
};

#endif /* XERYON_XD_CONROLLER_H */