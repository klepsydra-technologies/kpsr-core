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
#include<algorithm>

#include <spdlog/spdlog.h>


#include <klepsydra/dds_core/dds_env.h>

kpsr::dds_mdlw::DDSEnv::DDSEnv(const std::string yamlFileName, std::string ddsKey,
                             dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
                             dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader)
    : _decorableEnv(new YamlEnvironment(yamlFileName))
    , _dataWriter(dataWriter)
    , _dataReader(dataReader)
    , _ddsKey(ddsKey)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
{
    _listener = new DDSConfigurationListener(ddsKey, this, _timestamp);
    _dataReader->listener(_listener, dds::core::status::StatusMask::data_available());
}
kpsr::dds_mdlw::DDSEnv::DDSEnv(YamlEnvironment * yamlEnvironment,
                             dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
                             dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader)
    : _decorableEnv(yamlEnvironment)
    , _dataWriter(dataWriter)
    , _dataReader(dataReader)
    , _timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
{
    _decorableEnv->getPropertyString("kpsr_dds_env_key", _ddsKey);
    _listener = new DDSConfigurationListener(_ddsKey, this, _timestamp);
    _dataReader->listener(_listener, dds::core::status::StatusMask::data_available());
}

void kpsr::dds_mdlw::DDSEnv::updateConfiguration(std::string configurationData) {
    _decorableEnv->reload(configurationData);
}

void kpsr::dds_mdlw::DDSEnv::getPropertyString(const std::string key, std::string & value) {
    _decorableEnv->getPropertyString(key, value);
}

void kpsr::dds_mdlw::DDSEnv::getPropertyInt(const std::string key, int & value) {
    _decorableEnv->getPropertyInt(key, value);
}

void kpsr::dds_mdlw::DDSEnv::getPropertyFloat(const std::string key, float & value) {
    _decorableEnv->getPropertyFloat(key, value);
}

void kpsr::dds_mdlw::DDSEnv::getPropertyBool(const std::string key, bool & value) {
    _decorableEnv->getPropertyBool(key, value);
}

void kpsr::dds_mdlw::DDSEnv::setPropertyString(const std::string key, const std::string value) {
    _decorableEnv->setPropertyString(key, value);
    publishConfiguration();
}

void kpsr::dds_mdlw::DDSEnv::setPropertyInt(const std::string key, const int & value) {
    _decorableEnv->setPropertyInt(key, value);
    publishConfiguration();
}

void kpsr::dds_mdlw::DDSEnv::setPropertyFloat(const std::string key, const float & value) {
    _decorableEnv->setPropertyFloat(key, value);
    publishConfiguration();
}

void kpsr::dds_mdlw::DDSEnv::setPropertyBool(const std::string key, const bool & value) {
    _decorableEnv->setPropertyBool(key, value);
    publishConfiguration();
}

void kpsr::dds_mdlw::DDSEnv::persist() {
    _decorableEnv->persist();
}

void kpsr::dds_mdlw::DDSEnv::publishConfiguration() {
    std::string configurationData = _decorableEnv->exportEnvironment();
    kpsr_dds_core::DDSEnvironmentData environmentData(_ddsKey, configurationData, _timestamp);
    _dataWriter->write(environmentData);
}

void kpsr::dds_mdlw::DDSConfigurationListener::on_data_available(dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> & dataReader) {
    spdlog::info("kpsr::dds_mdlw::DDSConfigurationListener::on_data_available.");
    auto samples =  dataReader.select().state(dds::sub::status::DataState::new_data()).read();
    std::for_each(samples.begin(), samples.end(),
                  [this](const rti::sub::LoanedSample<kpsr_dds_core::DDSEnvironmentData>& s) {
        if ((s.data().configurationKey() == _ddsKey)&&(s.data().sourceId() != _sourceId)) {
            spdlog::info("kpsr::dds_mdlw::DDSConfigurationListener::on_data_available. new data: {}", s.data().configurationData());
            this->_ddsEnv->updateConfiguration(s.data().configurationData());
        }
    });
}
