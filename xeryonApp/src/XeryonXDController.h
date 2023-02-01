#ifndef XERYON_XD_CONROLLER_H
#define XERYON_XD_CONROLLER_H

#include "asynMotorController.h"
#include "XeryonXDAxis.h"
#include "XeryonException.h"

#include <array>

#define XDstatString "STAT"
#define XDsspdString "SSPD"
#define XDeposString "EPOS"
#define XDdposString "DPOS"

#define XDindxString "INDX"

#define XDptolString "PTOL"
#define XDpto2String "PTO2"

#define XDfreqString "FREQ"

#define XDtestString "TEST"

/**
 * @class Exception class used for exceptions related to MicroEpsilon controller.
 */
class XeryonControllerException : public XeryonException
{
public:
    XeryonControllerException(const std::string &description) : XeryonException(description) {}
};

class epicsShareClass XDController : public asynMotorController
{
public:
    /** Creates a new XDController object.
     * \param[in] portName             The name of the asyn port that will be created for this driver
     * \param[in] XDPortName         The name of the drvAsynIPPPort that was created previously to connect to the XD controller
     * \param[in] numAxes              The number of axes that this controller supports
     * \param[in] movingPollPeriod     The time between polls when any axis is moving
     * \param[in] idlePollPeriod       The time between polls when no axis is moving
     */
    XDController(const char *portName, const char *XDPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

    /* These are the methods that we override from asynMotorDriver */

    /**
     * @brief Called when asyn clients call pasynInt32->write().
     * @details Extracts the function and axis number from pasynUser.
     * Sets the value in the parameter library.
     * For all other functions it calls asynMotorController::writeInt32.
     * Calls any registered callbacks for this pasynUser->reason and address.
     * @param[in] pasynUser asynUser structure that encodes the reason and address.
     * @param[in] value     Value to write.
     */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /**
     * @brief poll method
     * @note not implemented --> Thus, the poll of the axis will be used instead.
     */
    // asynStatus poll();

    /* These are the methods that we override from asynMotorDriver */
    /**
     * @brief Reports on status of the driver
     * @param[in] fp The file pointer on which report information will be written
     * @param[in] level The level of report detail desired
     *
     * If details > 0 then information is printed about each axis.
     * After printing controller-specific information calls asynMotorController::report()
     */
    void report(FILE *fp, int level);

    /**
     * @brief Returns a pointer to an XDMotorAxis object.
     * @details Returns NULL if the axis number encoded in pasynUser is invalid.
     * @param[in] pasynUser asynUser structure that encodes the axis index number.
     */
    XDAxis *getAxis(asynUser *pasynUser);

    /**
     * @brief Returns a pointer to an XDMotorAxis object.
     * @details Returns NULL if the axis number encoded in pasynUser is invalid.
     * @param[in] axisNo Axis index number.
     */
    XDAxis *getAxis(int axisNo);

    /* ==== */
    std::string controllerName;

    void setParameter(XDController *device, const int &axisNo, const std::string &cmd, const std::string &payload);

    void setParameter(XDController *device, const std::string &cmd, const std::string &payload){
        std::cout << "==> setParameter w/o axis | w/ payload" << std::endl;
        setParameter(device, -1, cmd, payload);
    };
    void setParameter(XDController *device, const std::string &cmd){
        setParameter(device, -1, cmd, 0);
    };

    void getParameter(XDController *device, const int &axisNo, const std::string &cmd, std::string &reply);

    void getParameter(XDController *device, const std::string &cmd, std::string &reply) {
        std::cout << "==> getParameter w/o axis" << std::endl;
        getParameter(device, -1, cmd, reply);
    };

    std::shared_ptr<XDAxis> getAxisPointer(int axisNo) { return controllerAxes.at(axisNo); };

    std::array<std::shared_ptr<XDAxis>, 12> controllerAxes;

private:
    /**
     * @brief arrays for axis letters in the controller
     * @details for future use.
     * @todo move to other place and handle controller setup at instanciation
    // std::array<std::string, 3> XD_M_Axes{{"X", "Y", "Z"}};
    // std::array<std::string, 12> XD_19_Axes{{"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"}};
    */

protected:
    int statrb_; /**< axis status word readback */
#define FIRST_XD_PARAM statrb_
    int indx_;   /**< issue index search command */
    int ptol_;   /**< positioning tolerance 1 */
    int pto2_;   /**< positioning tolerance 2 */
    int test_;   /**< test LEDs (XD-M and XD-19) */
    int freqrb_; /**< Excitation frequency currently in use */
    // int ofrqrb_; /**< Optimal frequency as determined by FFRQ */
    // int currrb_; /**< Current consumed by the piezomotor */
    int timerb_; /**< Time stamp: resolution 0.1 ms */
    int eposrb_; /**< axis encoder readback */
    int dposrb_; /**< axis target position readback */
    int sspdrb_; /**< axis velocity setpoiny readback */
#define LAST_XD_PARAM sspdrb_
#define NUM_XD_PARAMS (&LAST_XD_PARAM - &FIRST_XD_PARAM + 1)

    friend class XDAxis;
};

/**
 * @class ControllerHolder, based on a driver developed by Tadej Humar (Paul Scherrer Institute, Switzerland)
 */
class ControllerHolder
{
public:
    static ControllerHolder &getInstance()
    {
        static ControllerHolder instance; // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

    /**
     * @brief Check and adds a XDController if needed.
     * @note Not thread safe but it is not needed anyway since configuration is created before IOC starts.
     * Since IOC is started after all controllers are added add and get controller functions will not be called at the same time.
     *
     * @param[in] portName          The name of the asyn port that will be created for this driver
     * @param[in] XDPortName        The name of the drvAsynIPPPort that was created previously to connect to the XD controller
     * @param[in] numAxes           The number of axes that this controller supports
     * @param[in] movingPollPeriod  The time in ms between polls when any axis is moving
     * @param[in] idlePollPeriod    The time in ms between polls when no axis is moving
     */
    void addController(const std::string &portName, const std::string &XDPortName, const uint16_t numAxes, const double movingPollPeriod, const double idlePollPeriod);

    /**
     * @brief Returns the controller shared_ptr under the provided name.
     * If the name does not exist (should never be the case if MACROS are used) exception is thrown. (std::out_of_range)
     *
     * @param name Name of the controller.
     */
    std::shared_ptr<XDController> getController(const std::string &name);

private:
    ControllerHolder(){};
    ControllerHolder(ControllerHolder const &) = delete;
    void operator=(ControllerHolder const &) = delete;
    std::unordered_map<std::string, std::shared_ptr<XDController>> controllerMap; //!< Container.
};

#endif /* XERYON_XD_CONROLLER_H */