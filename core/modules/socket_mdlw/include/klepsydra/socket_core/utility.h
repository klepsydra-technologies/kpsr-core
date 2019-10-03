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

#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <sstream>

namespace kpsr
{
namespace socket_mdlw
{

template<typename... Args>
/**
 * @brief print
 * @param s
 * @param args
 * @return
 */
int print(std::ostream& s, Args&... args)
{
    using Expander = int[];
    return Expander{ 0, ((s << std::forward<Args>(args)), 0)...}[0];
}

template<typename... Args>
/**
 * @brief buildStringFromParts
 * @param args
 * @return
 */
std::string buildStringFromParts(Args const&... args)
{
    std::stringstream msg;
    print(msg, args...);
    return msg.str();
}

template<typename... Args>
/**
 * @brief buildErrorMessage
 * @param args
 * @return
 */
std::string buildErrorMessage(Args const&... args)
{
    return buildStringFromParts(args...);
}

}
}

#endif

