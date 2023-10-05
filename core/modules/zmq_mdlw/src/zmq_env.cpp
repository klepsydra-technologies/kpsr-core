// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <chrono>
#include <string.h>

#include <spdlog/spdlog.h>

#include <klepsydra/core/configuration_environment.h>
#include <klepsydra/zmq_core/zmq_env.h>

static const std::string DEFAULT_ZMQ_TOPIC_NAME = "DEFAULT_ZMQ_TOPIC_NAME";
static const std::string DEFAULT_ZMQ_ENV_KEY = "DEFAULT_ZMQ_ENV_KEY";
static const int DEFAULT_ZMQ_ENV_POOL_PERIOD = 1000;

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(const std::string &configurationFileName,
                               std::string zmqKey,
                               std::string topicName,
                               int pollPeriod,
                               zmq::socket_t &zmqPublisher,
                               zmq::socket_t &zmqSubscriber,
                               const std::string &rootNode)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(configurationFileName.empty()
                        ? new ConfigurationEnvironment()
                        : new ConfigurationEnvironment(configurationFileName, rootNode))
    , _zmqPublisher(zmqPublisher)
    , _topicName(topicName)
    , _zmqKey(zmqKey)
    , _pollPeriod(pollPeriod)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count())
    , mapper()
    , _isEnvLocal(true)
{
    _poller = new ZMQConfigurationPoller(zmqKey, this, _timestamp, _pollPeriod);
    _poller->start();
}

kpsr::zmq_mdlw::ZMQEnv::~ZMQEnv()
{
    _poller->stop();
    delete _poller;
    if (_isEnvLocal) {
        delete _decorableEnv;
    }
}

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(ConfigurationEnvironment *configurationEnvironment,
                               zmq::socket_t &zmqPublisher,
                               zmq::socket_t &zmqSubscriber,
                               const std::string &rootNode)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(configurationEnvironment)
    , _zmqPublisher(zmqPublisher)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count())
    , mapper()
    , _isEnvLocal(false)
{
    _decorableEnv->getPropertyString("kpsr_zmq_env_topic_name",
                                     _topicName,
                                     DEFAULT_ZMQ_TOPIC_NAME,
                                     rootNode);
    _decorableEnv->getPropertyString("kpsr_zmq_env_key", _zmqKey, DEFAULT_ZMQ_ENV_KEY, rootNode);
    _decorableEnv->getPropertyInt("kpsr_zmq_env_poll_period",
                                  _pollPeriod,
                                  DEFAULT_ZMQ_ENV_POOL_PERIOD,
                                  rootNode);

    _poller = new ZMQConfigurationPoller(_zmqKey, this, _timestamp, _pollPeriod);
    _poller->start();
}

bool kpsr::zmq_mdlw::ZMQEnv::updateConfiguration(const std::string &configurationData)
{
    return _decorableEnv->updateConfiguration(configurationData);
}

bool kpsr::zmq_mdlw::ZMQEnv::getPropertyString(const std::string &key,
                                               std::string &value,
                                               const std::string &defaultValue,
                                               const std::string &rootNode)
{
    return _decorableEnv->getPropertyString(key, value, defaultValue, rootNode);
}

bool kpsr::zmq_mdlw::ZMQEnv::getPropertyInt(const std::string &key,
                                            int &value,
                                            const int defaultValue,
                                            const std::string &rootNode)
{
    return _decorableEnv->getPropertyInt(key, value, defaultValue, rootNode);
}

bool kpsr::zmq_mdlw::ZMQEnv::getPropertyFloat(const std::string &key,
                                              float &value,
                                              const float defaultValue,
                                              const std::string &rootNode)
{
    return _decorableEnv->getPropertyFloat(key, value, defaultValue, rootNode);
}

bool kpsr::zmq_mdlw::ZMQEnv::getPropertyBool(const std::string &key,
                                             bool &value,
                                             const bool defaultValue,
                                             const std::string &rootNode)
{
    return _decorableEnv->getPropertyBool(key, value, defaultValue, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyString(const std::string &key,
                                               const std::string &value,
                                               const std::string &rootNode)
{
    _decorableEnv->setPropertyString(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyInt(const std::string &key,
                                            const int &value,
                                            const std::string &rootNode)
{
    _decorableEnv->setPropertyInt(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyFloat(const std::string &key,
                                              const float &value,
                                              const std::string &rootNode)
{
    _decorableEnv->setPropertyFloat(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyBool(const std::string &key,
                                             const bool &value,
                                             const std::string &rootNode)
{
    _decorableEnv->setPropertyBool(key, value, rootNode);
    publishConfiguration();
}

bool kpsr::zmq_mdlw::ZMQEnv::loadFile(const std::string &fileName, const std::string &nodeName)
{
    if (_decorableEnv->loadFile(fileName, nodeName)) {
        publishConfiguration();
        return true;
    } else {
        return false;
    }
}

void kpsr::zmq_mdlw::ZMQEnv::publishConfiguration()
{
    std::string configurationData = _decorableEnv->exportEnvironment();
    ZMQEnvironmentData environmentData(_zmqKey, configurationData, _timestamp);
    _zmqPublisher.send(zmq::const_buffer(_topicName.c_str(), _topicName.size()),
                       zmq::send_flags::sndmore);
    std::string serializeEnvironmentData;
    mapper.toMiddleware(environmentData, serializeEnvironmentData);
    _zmqPublisher.send(
        zmq::const_buffer(serializeEnvironmentData.c_str(), serializeEnvironmentData.size()));
}

kpsr::zmq_mdlw::ZMQConfigurationPoller::ZMQConfigurationPoller(const std::string &zmqKey,
                                                               ZMQEnv *zmqEnv,
                                                               long sourceId,
                                                               int pollPeriod)
    : _zmqKey(zmqKey)
    , _zmqEnv(zmqEnv)
    , _sourceId(sourceId)
    , _pollPeriod(pollPeriod)
    , _running(false)
{}

void kpsr::zmq_mdlw::ZMQConfigurationPoller::start()
{
    _running = true;
    _threadPoller = std::thread([this]() {
        while (_running) {
            poll();
        }
    });
}

void kpsr::zmq_mdlw::ZMQConfigurationPoller::stop()
{
    _running = false;
    if (_threadPoller.joinable()) {
        _threadPoller.join();
    }
}
void kpsr::zmq_mdlw::ZMQConfigurationPoller::poll()
{
    zmq::pollitem_t items[] = {{_zmqEnv->_zmqSubscriber, 0, ZMQ_POLLIN, 0}};
    if (zmq::poll(items, 1, _pollPeriod) == -1)
        return;

    if (items[0].revents & ZMQ_POLLIN) {
        zmq::message_t topicMsg;
        zmq::message_t content;
        _zmqEnv->_zmqSubscriber.recv(topicMsg);
        _zmqEnv->_zmqSubscriber.recv(content);
        std::string topic(static_cast<char *>(topicMsg.data()), topicMsg.size());
        std::string contentString(static_cast<char *>(content.data()), content.size());
        spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::on_data_available.");
        ZMQEnvironmentData environmentData;
        mapper.fromMiddleware(contentString, environmentData);
        if ((environmentData._configurationKey == _zmqKey) &&
            (environmentData._sourceId != _sourceId)) {
            spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::poll. new data: {}",
                         environmentData._configurationData);
            this->_zmqEnv->updateConfiguration(environmentData._configurationData);
        }
    }
}
