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

#ifndef CEREAL_BINARY_MAPPER_H
#define CEREAL_BINARY_MAPPER_H

#include <memory>
#include <streambuf>

#include <cereal/cereal.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/string.hpp>

#include <klepsydra/serialization/mapper.h>

using Base = std::basic_streambuf<char> *;

namespace kpsr
{
template <class T>
/**
 * @brief The Mapper<T, Base> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, Base>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const Base & message, T& event) {
        std::istream iss(message);
        cereal::PortableBinaryInputArchive archive(iss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T& event, Base & message) {
        std::ostream oss(message);
        {
            cereal::PortableBinaryOutputArchive archive(oss);
            archive(CEREAL_NVP(event));
        }
    }
};
}

#endif // CEREAL_BINARY_MAPPER_H
