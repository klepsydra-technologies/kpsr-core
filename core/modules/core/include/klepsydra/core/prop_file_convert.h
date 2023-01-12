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

#ifndef CONVERT_H
#define CONVERT_H

#include <exception>
#include <sstream>
#include <string>
#include <typeinfo>

namespace kpsr {
class PropertyFileConvert
{
public:
    template<typename T>
    static std::string T_to_string(T const &val)
    {
        std::ostringstream ostr;
        ostr << val;

        return ostr.str();
    }

    template<typename T>
    static T string_to_T(const std::string &val)
    {
        T returnVal;
        std::istringstream iss(val);
        std::string::size_type hexCode = val.find("0x", 0);
        if (hexCode == std::string::npos) {
            if (!(iss >> returnVal))
                throw std::invalid_argument("CFG: Not a valid " + (std::string) typeid(T).name() +
                                            " received!\n");
        } else {
            iss >> std::hex >> returnVal;
        }

        return returnVal;
    }
};
} // namespace kpsr

#endif
