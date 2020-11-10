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

#include <string.h>
#include <chrono>
#include <algorithm>

#include <spdlog/spdlog.h>

#include <klepsydra/zmq_core/zmq_env.h>

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(const std::string yamlFileName,
                             std::string zmqKey,
                             std::string topicName,
                             int pollPeriod,
                             zmq::socket_t & zmqPublisher,
                             zmq::socket_t & zmqSubscriber,
                             const std::string& rootNode)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(new YamlEnvironment(yamlFileName, rootNode))
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
    delete _poller;
}

kpsr::zmq_mdlw::ZMQEnv::ZMQEnv(YamlEnvironment * yamlEnvironment,
                               zmq::socket_t & zmqPublisher,
                               zmq::socket_t & zmqSubscriber,
                               const std::string& rootNode)
    : _zmqSubscriber(zmqSubscriber)
    , _decorableEnv(yamlEnvironment)
    , _zmqPublisher(zmqPublisher)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
{
    _decorableEnv->getPropertyString("kpsr_zmq_env_topic_name", _topicName, rootNode);
    _decorableEnv->getPropertyString("kpsr_zmq_env_key", _zmqKey, rootNode);
    _decorableEnv->getPropertyInt("kpsr_zmq_env_poll_period", _pollPeriod, rootNode);

    _poller = new ZMQConfigurationPoller(_zmqKey, this, _timestamp, _pollPeriod);
    _poller->start();
}

void kpsr::zmq_mdlw::ZMQEnv::updateConfiguration(const std::string & configurationData) {
    _decorableEnv->updateConfiguration(configurationData);
}

void kpsr::zmq_mdlw::ZMQEnv::updateConfiguration(const std::string & configurationData, const std::string & rootNode) {
    _decorableEnv->updateConfiguration(configurationData, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyString(const std::string & key, std::string & value, const std::string & rootNode) {
    _decorableEnv->getPropertyString(key, value, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyInt(const std::string & key, int & value, const std::string & rootNode) {
    _decorableEnv->getPropertyInt(key, value, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyFloat(const std::string & key, float & value, const std::string & rootNode) {
    _decorableEnv->getPropertyFloat(key, value, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::getPropertyBool(const std::string & key, bool & value, const std::string & rootNode) {
    _decorableEnv->getPropertyBool(key, value, rootNode);
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode) {
    _decorableEnv->setPropertyString(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyInt(const std::string & key, const int & value, const std::string & rootNode) {
    _decorableEnv->setPropertyInt(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode) {
    _decorableEnv->setPropertyFloat(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode) {
    _decorableEnv->setPropertyBool(key, value, rootNode);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::loadFile(const std::string & fileName, const std::string & nodeName) {
    _decorableEnv->loadFile(fileName, nodeName);
    publishConfiguration();
}

void kpsr::zmq_mdlw::ZMQEnv::publishConfiguration() {
    std::string configurationData = _decorableEnv->exportEnvironment();
    ZMQEnvironmentData environmentData(_zmqKey, configurationData, _timestamp);
    _zmqPublisher.send(zmq::const_buffer(_topicName.c_str(), _topicName.size()), zmq::send_flags::sndmore);
    std::string serializeEnvironmentData;
    mapper.toMiddleware(environmentData, serializeEnvironmentData);
    _zmqPublisher.send(zmq::const_buffer(serializeEnvironmentData.c_str(), serializeEnvironmentData.size()));
}

kpsr::zmq_mdlw::ZMQConfigurationPoller::ZMQConfigurationPoller(const std::string & zmqKey,
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
        zmq::message_t topicMsg;
        zmq::message_t content;
        _zmqEnv->_zmqSubscriber.recv(topicMsg);
        _zmqEnv->_zmqSubscriber.recv(content);
        std::string topic(static_cast<char*>(topicMsg.data()), topicMsg.size());
        std::string contentString(static_cast<char*>(content.data()), content.size());
        spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::on_data_available.");
        ZMQEnvironmentData environmentData;
        mapper.fromMiddleware(contentString, environmentData);
        if ((environmentData._configurationKey == _zmqKey) && (environmentData._sourceId != _sourceId)) {
            spdlog::info("kpsr::zmq_mdlw::ZMQConfigurationPoller::poll. new data: {}", environmentData._configurationData);
            this->_zmqEnv->updateConfiguration(environmentData._configurationData);
        }
    }
}
