/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

#ifndef CONVERT_H
#define CONVERT_H

#include <string>
#include <sstream>
#include <typeinfo>
#include <exception>

namespace kpsr
{
class PropertyFileConvert
{
public:
    template <typename T>
    static std::string T_to_string(T const &val) {
        std::ostringstream ostr;
        ostr << val;

        return ostr.str();
    }

    template <typename T>
    static T string_to_T(const std::string & val) {
        T returnVal;
        std::istringstream iss(val);
        std::string::size_type hexCode = val.find( "0x", 0 );
        if (hexCode == std::string::npos) {
            if (!(iss >> returnVal))
                throw std::invalid_argument("CFG: Not a valid " + (std::string)typeid(T).name() + " received!\n");
        }
        else {
            iss >> std::hex >> returnVal;
        }

        return returnVal;
    }
};
}

#endif
