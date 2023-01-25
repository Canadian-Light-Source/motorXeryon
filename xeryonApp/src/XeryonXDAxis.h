#ifndef XERYON_XD_AXIS_H
#define XERYON_XD_AXIS_H

#include "asynMotorAxis.h"

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

private:
  XDController *pC_; /**< Pointer to the asynMotorController to which this axis belongs.
                      *   Abbreviated because it is used very frequently */
  int channel_;
  asynStatus comStatus_;

  friend class XDController;
};

#endif /* XERYON_XD_AXIS_H */