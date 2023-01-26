#ifndef XERYON_XD_AXIS_H
#define XERYON_XD_AXIS_H

#include <array>
#include <string>
#include <iostream>

#include "asynMotorAxis.h"
#include "XeryonAxis.h" // convenience class

class XDController;

class epicsShareClass XDAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  XDAxis(class XDController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus poll(bool *moving);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  // asynStatus setPosition(double position);
  // XD specific methods
  double decodeReply(std::string buf, std::string delimiter)
  {
    // std::string token = buf.substr(buf.find(delimiter)+1);
    // std::cout << token << std::endl;
    return std::stod (buf.substr(buf.find(delimiter)+1));
  };

  double decodeReply(std::string buf) { 
    return decodeReply(buf, "="); };

  // template<typename T> void decodeReply2(T const& t, std::string buf){}


private:
  XDController *pC_; /**< Pointer to the asynMotorController to which this axis belongs.
                      *   Abbreviated because it is used very frequently */
  int channel_;
  asynStatus comStatus_;
  // XeryonAxis *axisContainer; /**< Container

  friend class XDController;
};

#endif /* XERYON_XD_AXIS_H */
