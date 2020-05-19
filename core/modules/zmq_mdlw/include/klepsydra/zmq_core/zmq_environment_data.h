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

#ifndef ZMQ_ENVIRONMENT_DATA_H
#define ZMQ_ENVIRONMENT_DATA_H

#include <string>

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>

namespace kpsr
{
namespace zmq_mdlw
{
/**
 * @brief The ZMQEnvironmentData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
struct ZMQEnvironmentData {
public:
    /**
     * @brief _configurationKey
     */
    std::string _configurationKey;

    /**
     * @brief _configurationData
     */
    std::string _configurationData;

    /**
     * @brief _sourceId
     */
    long _sourceId;

    /**
     * @brief ZMQEnvironmentData
     */
    ZMQEnvironmentData() {}

    /**
     * @brief ZMQEnvironmentData
     * @param configurationKey
     * @param configurationData
     * @param sourceId
     */
    ZMQEnvironmentData(const std::string & configurationKey, const std::string & configurationData, long sourceId)
        : _configurationKey(configurationKey)
        , _configurationData(configurationData)
        , _sourceId(sourceId)
    {}
};
}
}

namespace cereal {
template<class Archive>
/**
 * @brief serialize
 * @param archive
 * @param event
 */
void serialize(Archive & archive, kpsr::zmq_mdlw::ZMQEnvironmentData & event)
{
    archive( CEREAL_NVP(event._configurationKey),
             CEREAL_NVP(event._configurationData),
             CEREAL_NVP(event._sourceId) );
}
}

#endif // ZMQ_ENVIRONMENT_DATA_H
