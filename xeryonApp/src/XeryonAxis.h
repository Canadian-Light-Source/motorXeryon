#ifndef XERYON_AXIS_H
#define XERYON_AXIS_H

#include <array>
#include <string>
#include <iostream>

class XeryonAxis
{
public:
    XeryonAxis(){};
    void setStatus(int s) { status = s; std::cout << status << std::endl;};
    int getStatus() { return status; };

private:
    uint status;
};

#endif /* XERYON_AXIS_H */
