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

#ifndef STRING_RESIZER
#define STRING_RESIZER

#include <string>

namespace kpsr {
namespace socket_mdlw {
/**
 * @brief The StringSizer class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    std::string&    stringData;
    std::size_t&    currentSize;
public:
    /**
     * @brief StringSizer
     * @param stringData
     * @param currentSize
     */
    StringSizer(std::string& stringData, std::size_t& currentSize)
        : stringData(stringData)
        , currentSize(currentSize) {
        stringData.resize(stringData.capacity());
    }

    ~StringSizer() {
        stringData.resize(currentSize);
    }

    /**
     * @brief incrementSize
     * @param amount
     */
    void incrementSize(std::size_t amount) {
        currentSize += amount;
    }
};
}
}

#endif // STRING_RESIZER

