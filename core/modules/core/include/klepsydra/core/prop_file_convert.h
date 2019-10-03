/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
    static T string_to_T(std::string const &val) {
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
