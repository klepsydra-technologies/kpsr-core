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

#ifndef ZMQ_ENV_H
#define ZMQ_ENV_H

#include <zmq.hpp>

#include <thread>

#include <klepsydra/serialization/json_cereal_mapper.h>

#include <klepsydra/core/yaml_environment.h>

#include <klepsydra/zmq_core/zmq_environment_data.h>

namespace kpsr
{
namespace zmq_mdlw
{
class ZMQEnv;
/**
 * @brief The ZMQConfigurationPoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class ZMQConfigurationPoller
{
public:
    ZMQConfigurationPoller(std::string zmqKey, ZMQEnv * zmqEnv, long sourceId, int pollPeriod);

    void poll();

    void start();

    void stop();

private:
    std::string _zmqKey;
    ZMQEnv * _zmqEnv;
    long _sourceId;
    int _pollPeriod;
    bool _running;
    kpsr::Mapper<ZMQEnvironmentData, std::string> mapper;
    std::thread _threadPoller;
};

/**
 * @brief The ZMQEnv class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-composition
 *
 * @details ZMQ realm implementation of the Klepsydra Environment interface. It also decorate the YALM persistent environment.
 *
 */
class ZMQEnv : public Environment
{
public:
    /**
     * @brief ZMQEnv
     * @param yamlFileName file name to persist to. If empty, no persistent service.
     * @param zmqKey key to identify environment changes messages.
     * @param topicName ZMQ topic where environment data is provided
     * @param zmqPublisher ZMQ specific object
     * @param zmqSubscriber ZMQ specific object
     */
    ZMQEnv(const std::string yamlFileName,
           std::string zmqKey,
           std::string topicName,
           int pollPeriod,
           zmq::socket_t & zmqPublisher,
           zmq::socket_t & zmqSubscriber);

    /**
     * @brief ZMQEnv
     * @param yamlEnvironment
     * @param zmqPublisher
     * @param zmqSubscriber
     */
    ZMQEnv(YamlEnvironment * yamlEnvironment,
           zmq::socket_t & zmqPublisher,
           zmq::socket_t & zmqSubscriber);

    ~ZMQEnv();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string key, std::string & value) override;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string key, int & value) override;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string key, float & value) override;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string key, bool & value) override;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string key, const std::string value) override;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string key, const int & value) override;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string key, const float & value) override;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string key, const bool & value) override;

    void persist() override;

    /**
     * @brief updateConfiguration
     * @param configurationData
     */
    void updateConfiguration(std::string configurationData);

    zmq::socket_t & _zmqSubscriber;

private:
    void publishConfiguration();

    YamlEnvironment * _decorableEnv;

    zmq::socket_t & _zmqPublisher;
    std::string _topicName;
    std::string _zmqKey;
    int _pollPeriod;
    long _timestamp;
    ZMQConfigurationPoller * _poller;
    kpsr::Mapper<ZMQEnvironmentData, std::string> mapper;

};
}
}
#endif
