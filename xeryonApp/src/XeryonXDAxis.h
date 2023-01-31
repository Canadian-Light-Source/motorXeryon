#ifndef XERYON_XD_AXIS_H
#define XERYON_XD_AXIS_H

#include <array>
#include <string>
#include <iostream>
#include <memory>

#include "asynMotorAxis.h"
#include "XeryonAxis.h" // convenience class

class XDController;

class epicsShareClass XDAxis : public XeryonAxis, public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */

  /**
   * @brief Creates a new XDAxis object.
   * @details Initializes register numbers, etc.
   * @param[in] pC Pointer to the XDController to which this axis belongs.
   * @param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
   */
  XDAxis(class XDController *pC, int axis);

  /**
   * @brief Reports on status of the driver
   * @details If details > 0 then information is printed about each axis.
   * After printing controller-specific information calls asynMotorController::report()
   * @param[in] fp The file pointer on which report information will be written
   * @param[in] level The level of report detail desired
   */
  void report(FILE *fp, int level);

  /**
   * @brief Polls the axis.
   * @details This function reads the controller position, encoder position, the limit status, the moving status,
   * the drive power-on status and positioner type.
   * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
   * and then calls callParamCallbacks() at the end.
   * @param[out] moving A flag that is set indicating that the axis is moving (1) or done (0).
   */
  asynStatus poll(bool *moving);

  /**
   * @brief Move the axis.
   * @param[in] postion The new position of the axis
   * @param[in] relative Whether the move is relative or absolute
   * @param[in] min_velocity The minimum allowed velocity
   * @param[in] max_velocity The maximum allowed velocity
   * @param[in] acceleration The desiered acceleration // disregarded in this driver for the moment
   */
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);

  /**
   * @brief Find reference position.
   * @param[in] min_velocity The minimum allowed velocity
   * @param[in] max_velocity The maximum allowed velocity
   * @param[in] acceleration The desiered acceleration // disregarded in this driver for the moment
   * @param[in] forwards direction of movement
   * @return
   */
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);

  /**
   * @brief Stop the axis.
   * @details This method will force the driver output to zero. No controlled deceleration is performed.
   * @param[in] acceleration The desiered acceleration // disregarded in this driver for the moment
   */
  asynStatus stop(double acceleration);

  /**
   * @brief Set the current position.
   * @note This method is not implemented yet
   */
  asynStatus setPosition(double position){};

  // XD specific methods
  /**
   * @brief Decode the controller reply.
   * @param[in] buf the reply buffer
   * @param[in] delimiter the delimiter
   * @todo move to ZeryonAxis.h
   */
  double decodeReply(std::string buf, std::string delimiter)
  {
    return std::stod(buf.substr(buf.find(delimiter) + 1));
  };

  /**
   * @brief decode the controller reply
   * @param[in] buf the reply buffer
   * @overload decodeReply with no delimiter, default is "="
   * @todo move to ZeryonAxis.h
   */
  double decodeReply(std::string buf)
  {
    return decodeReply(buf, "=");
  };

private:
  XDController *pC_; /**< Pointer to the asynMotorController to which this axis belongs.
                      *   Abbreviated because it is used very frequently */
  int channel_;
  asynStatus comStatus_;

  friend class XDController;
};

#endif /* XERYON_XD_AXIS_H */
