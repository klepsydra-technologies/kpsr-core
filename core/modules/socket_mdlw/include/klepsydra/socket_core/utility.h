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

