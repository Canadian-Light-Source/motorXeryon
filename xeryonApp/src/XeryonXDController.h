#ifndef XERYON_XD_CONROLLER_H
#define XERYON_XD_CONROLLER_H

#include "asynMotorController.h"
#include "XeryonXDAxis.h"

#define XDstatString "STAT"
#define XDsspdString "SSPD"
#define XDeposString "EPOS"
#define XDdposString "DPOS"

#define XDindxString "INDX"

#define XDptolString "PTOL"
#define XDpto2String "PTO2"

#define XDtestString "TEST"

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
    int statrb_; /**< axis status word readback */
#define FIRST_XD_PARAM statrb_
    int indx_; /**< issue index search command */
    int ptol_; /**< positioning tolerance 1 */
    int pto2_; /**< positioning tolerance 2 */
    int test_; /**< test LEDs (XD-M and XD-19) */
    int eposrb_; /**< axis encoder readback */
    int dposrb_; /**< axis target position readback */
    int sspdrb_; /**< axis velocity setpoiny readback */
#define LAST_XD_PARAM sspdrb_
#define NUM_XD_PARAMS (&LAST_XD_PARAM - &FIRST_XD_PARAM + 1)

    friend class XDAxis;
};

#endif /* XERYON_XD_CONROLLER_H */