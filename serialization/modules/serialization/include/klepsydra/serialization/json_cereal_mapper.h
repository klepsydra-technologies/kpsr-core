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

#ifndef CEREAL_JSON_MAPPER_H
#define CEREAL_JSON_MAPPER_H

#include <sstream>

#include <cereal/cereal.hpp>
#include <cereal/archives/json.hpp>

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template <class T>
/**
 * @brief The Mapper<T, std::string> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, std::string>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std::string& message, T& event) {
        std::stringstream ss;
        ss.str(message);
        cereal::JSONInputArchive archive(ss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T& event, std::string& message) {
        std::stringstream ss;
        {
            cereal::JSONOutputArchive archive( ss );
            archive(CEREAL_NVP(event));
        }
        message = ss.str();
    }
};
}

#endif // CEREAL_JSON_MAPPER_H
