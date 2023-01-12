/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <sstream>
#include <string>

namespace kpsr {
namespace socket_mdlw {

template<typename... Args>
/**
 * @brief print
 * @param s
 * @param args
 * @return
 */
int print(std::ostream &s, Args &...args)
{
    using Expander = int[];
    return Expander{0, ((s << std::forward<Args>(args)), 0)...}[0];
}

template<typename... Args>
/**
 * @brief buildStringFromParts
 * @param args
 * @return
 */
std::string buildStringFromParts(Args const &...args)
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
std::string buildErrorMessage(Args const &...args)
{
    return buildStringFromParts(args...);
}

} // namespace socket_mdlw
} // namespace kpsr

#endif
