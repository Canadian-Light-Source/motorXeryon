//
// Created by Tadej Humar (PSI) on 7/28/15.
// Modified by Niko Kivel (CLS) on 2023-Feb-01
//

#ifndef XERYON_EXCEPTION_H
#define XERYON_EXCEPTION_H

#include <stdexcept>

/**
 * Base exception class for Xeryon XD controllers.
 */
class XeryonException : public std::runtime_error
{
public:
    XeryonException(const std::string &description) : std::runtime_error(description) {}
};

#endif // XERYON_EXCEPTION_H
