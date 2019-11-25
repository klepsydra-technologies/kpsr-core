/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file ‘LICENSE.md’, which is part of this source code package.
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

#include <string.h>
#include <chrono>
#include <algorithm>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include <klepsydra/zmq_core/zhelpers.hpp>
#include <klepsydra/zmq_core/zmq_env.h>

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(const std::string yamlFileName,
                             std::string zmqKey,
                             std::string topicName,
                             int pollPeriod,
                             zmq::socket_t & zmqPublisher,
                             zmq::socket_t & zmqSubscriber)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(new YamlEnvironment(yamlFileName))
    , _zmqPublisher(zmqPublisher)
    , _topicName(topicName)
    , _zmqKey(zmqKey)
    , _pollPeriod(pollPeriod)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
{
    _poller = new ZMQConfigurationPoller(zmqKey, this, _timestamp, _pollPeriod);
    _poller->start();
}

kpsr::zmq_mdlw::ZMQEnv::~ZMQEnv() {
    _poller->stop();
}

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(YamlEnvironment * yamlEnvironment,
                             zmq::socket_t & zmqPublisher,
                             zmq::socket_t & zmqSubscriber)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(yamlEnvironment)
    , _zmqPublisher(zmqPublisher)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
{
    _decorableEnv->getPropertyString("kpsr_zmq_env_topic_name", _topicName);
    _decorableEnv->getPropertyString("kpsr_zmq_env_key", _zmqKey);
    _decorableEnv->getPropertyInt("kpsr_zmq_env_poll_period", _pollPeriod);

    _poller = new ZMQConfigurationPoller(_zmqKey, this, _timestamp, _pollPeriod);
}

void kpsr::zmq_mdlw::ZMQEnv::updateConfiguration(std::string configurationData) {
    _decorableEnv->reload(configurationData);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyString(const std::string key, std::string & value) {
    _decorableEnv->getPropertyString(key, value);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyInt(const std::string key, int & value) {
    _decorableEnv->getPropertyInt(key, value);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyFloat(const std::string key, float & value) {
    _decorableEnv->getPropertyFloat(key, value);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyBool(const std::string key, bool & value) {
    _decorableEnv->getPropertyBool(key, value);
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyString(const std::string key, const std::string value) {
    _decorableEnv->setPropertyString(key, value);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyInt(const std::string key, const int & value) {
    _decorableEnv->setPropertyInt(key, value);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyFloat(const std::string key, const float & value) {
    _decorableEnv->setPropertyFloat(key, value);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyBool(const std::string key, const bool & value) {
    _decorableEnv->setPropertyBool(key, value);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::persist() {
    _decorableEnv->persist();
}

void kpsr::zmq_mdlw::ZMQEnv::publishConfiguration() {
    std::string configurationData = _decorableEnv->exportEnvironment();
    ZMQEnvironmentData environmentData(_zmqKey, configurationData, _timestamp);
    s_sendmore (_zmqPublisher, _topicName.c_str());
    std::string serializeEnvironmentData;
    mapper.toMiddleware(environmentData, serializeEnvironmentData);
    s_send (_zmqPublisher, serializeEnvironmentData.c_str());
}

kpsr::zmq_mdlw::ZMQConfigurationPoller::ZMQConfigurationPoller(std::string zmqKey,
                                                             ZMQEnv * zmqEnv,
                                                             long sourceId,
                                                             int pollPeriod)
    : _zmqKey(zmqKey)
    , _zmqEnv(zmqEnv)
    , _sourceId(sourceId)
    , _pollPeriod(pollPeriod)
    , _running(false)
{}

void kpsr::zmq_mdlw::ZMQConfigurationPoller::start() {
    _running = true;
    _threadPoller = std::thread([this]() {
        while (_running) {
            poll();
        }

    });
}

void kpsr::zmq_mdlw::ZMQConfigurationPoller::stop() {
    _running = false;
    if (_threadPoller.joinable()) {
        _threadPoller.join();
    }
}
void kpsr::zmq_mdlw::ZMQConfigurationPoller::poll() {
    zmq::pollitem_t items [] = {
        { _zmqEnv->_zmqSubscriber, 0, ZMQ_POLLIN, 0 }
    };
    if (zmq::poll(items, 1, _pollPeriod) == -1)
        return;

    if (items[0].revents & ZMQ_POLLIN) {
        std::string topic = s_recv (_zmqEnv->_zmqSubscriber);
        std::string contents = s_recv (_zmqEnv->_zmqSubscriber);
        spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::on_data_available.");
        ZMQEnvironmentData environmentData;
        mapper.fromMiddleware(contents, environmentData);
        if ((environmentData._configurationKey == _zmqKey) && (environmentData._sourceId != _sourceId)) {
            spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::poll. new data: {}", environmentData._configurationData);
            this->_zmqEnv->updateConfiguration(environmentData._configurationData);
        }
    }
}
