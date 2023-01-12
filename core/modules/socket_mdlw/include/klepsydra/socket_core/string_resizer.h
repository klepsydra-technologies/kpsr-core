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

#ifndef STRING_RESIZER
#define STRING_RESIZER

#include <string>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The StringSizer class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-sockets-internal
 *
 * @details Utility class used by getMessage() to open the string upto capacity size.
 * Then on destruction resize to the actual size of the string.
 *
 */
class StringSizer
{
    std::string &stringData;
    std::size_t &currentSize;

public:
    /**
     * @brief StringSizer
     * @param stringData
     * @param currentSize
     */
    StringSizer(std::string &stringData, std::size_t &currentSize)
        : stringData(stringData)
        , currentSize(currentSize)
    {
        stringData.resize(stringData.capacity());
    }

    ~StringSizer() { stringData.resize(currentSize); }

    /**
     * @brief incrementSize
     * @param amount
     */
    void incrementSize(std::size_t amount) { currentSize += amount; }
};
} // namespace socket_mdlw
} // namespace kpsr

#endif // STRING_RESIZER
