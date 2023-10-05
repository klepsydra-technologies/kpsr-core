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

#ifndef ZMQ_ENV_H
#define ZMQ_ENV_H

#include <zmq.hpp>

#include <future>

#include <klepsydra/serialization/json_cereal_mapper.h>

#include <klepsydra/sdk/environment.h>
#include <klepsydra/zmq_core/zmq_environment_data.h>

namespace kpsr {

class ConfigurationEnvironment;
namespace zmq_mdlw {
class ZMQEnv;
/**
 * @brief The ZMQConfigurationPoller class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class ZMQConfigurationPoller
{
public:
    ZMQConfigurationPoller(const std::string &zmqKey, ZMQEnv *zmqEnv, long sourceId, int pollPeriod);

    void poll();

    void start();

    void stop();

private:
    std::string _zmqKey;
    ZMQEnv *_zmqEnv;
    long _sourceId;
    int _pollPeriod;
    bool _running;
    kpsr::Mapper<ZMQEnvironmentData, std::string> mapper;
    std::thread _threadPoller;
};

/**
 * @brief The ZMQEnv class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
     * @param configurationFileName file name to persist to. If empty, no persistent service.
     * @param zmqKey key to identify environment changes messages.
     * @param topicName ZMQ topic where environment data is provided
     * @param zmqPublisher ZMQ specific object
     * @param zmqSubscriber ZMQ specific object
     */
    ZMQEnv(const std::string &configurationFileName,
           std::string zmqKey,
           std::string topicName,
           int pollPeriod,
           zmq::socket_t &zmqPublisher,
           zmq::socket_t &zmqSubscriber,
           const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief ZMQEnv
     * @param configurationEnvironment
     * @param zmqPublisher
     * @param zmqSubscriber
     */
    ZMQEnv(ConfigurationEnvironment *configurationEnvironment,
           zmq::socket_t &zmqPublisher,
           zmq::socket_t &zmqSubscriber,
           const std::string &rootNode = kpsr::DEFAULT_ROOT);

    ~ZMQEnv();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    bool getPropertyString(const std::string &key,
                           std::string &value,
                           const std::string &defaultValue = "",
                           const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    bool getPropertyInt(const std::string &key,
                        int &value,
                        const int defaultValue = 0,
                        const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    bool getPropertyFloat(const std::string &key,
                          float &value,
                          const float defaultValue = 0.0f,
                          const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    bool getPropertyBool(const std::string &key,
                         bool &value,
                         const bool defaultValue = false,
                         const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string &key,
                           const std::string &value,
                           const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string &key,
                        const int &value,
                        const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string &key,
                          const float &value,
                          const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string &key,
                         const bool &value,
                         const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    bool loadFile(const std::string &fileName, const std::string &nodeName) override;

    /**
     * @brief updateConfiguration
     * @param configurationData
     */
    bool updateConfiguration(const std::string &configurationData);

    zmq::socket_t &_zmqSubscriber;

private:
    void publishConfiguration();

    ConfigurationEnvironment *_decorableEnv;

    zmq::socket_t &_zmqPublisher;
    std::string _topicName;
    std::string _zmqKey;
    int _pollPeriod;
    long _timestamp;
    ZMQConfigurationPoller *_poller;
    kpsr::Mapper<ZMQEnvironmentData, std::string> mapper;
    bool _isEnvLocal;
};
} // namespace zmq_mdlw
} // namespace kpsr
#endif
