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

#include <string.h>
#include <chrono>
#include<algorithm>

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
    std::cout << "kpsr::dds_mdlw::DDSConfigurationListener::on_data_available." << std::endl;
    auto samples =  dataReader.select().state(dds::sub::status::DataState::new_data()).read();
    std::for_each(samples.begin(), samples.end(),
                  [this](const rti::sub::LoanedSample<kpsr_dds_core::DDSEnvironmentData>& s) {
        if ((s.data().configurationKey() == _ddsKey)&&(s.data().sourceId() != _sourceId)) {
            std::cout << "kpsr::dds_mdlw::DDSConfigurationListener::on_data_available. new data: " << s.data().configurationData() << std::endl;
            this->_ddsEnv->updateConfiguration(s.data().configurationData());
        }
    });
}
