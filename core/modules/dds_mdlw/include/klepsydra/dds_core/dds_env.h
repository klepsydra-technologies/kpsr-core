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

#ifndef DDS_ENV_H
#define DDS_ENV_H

#include "dds/dds.hpp"
#include "dds_configuration_data.hpp"

#include <klepsydra/core/yaml_environment.h>

#include <klepsydra/dds_core/dds_environment_data.h>

namespace kpsr
{
namespace dds_mdlw
{
class DDSEnv;
/**
 * @brief The DDSConfigurationListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-internal
 */
class DDSConfigurationListener : public dds::sub::NoOpDataReaderListener<kpsr_dds_core::DDSEnvironmentData>
{
public:
    /**
     * @brief DDSConfigurationListener
     * @param ddsKey
     * @param ddsEnv
     * @param sourceId
     */
    DDSConfigurationListener(std::string ddsKey, DDSEnv * ddsEnv, long sourceId)
        : _ddsKey(ddsKey)
        , _ddsEnv(ddsEnv)
        , _sourceId(sourceId)
    {}


    virtual void on_data_available(dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> & dataReader);

private:
    std::string _ddsKey;
    DDSEnv * _ddsEnv;
    long _sourceId;
};

/**
 * @brief The DDSEnv class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-composition
 *
 * @details DDS realm implementation of the Klepsydra Environment interface. It also decorate the YAML persistent
 * environment. The following is an example of use:
@code
    // Create the DDS pub and sub to be used to broadcast configuration
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher pub(dp);
    dds::sub::Subscriber sub(dp);

    dds::topic::Topic<kpsr_dds_core::DDSEnvironmentData> topic(dp, "kpsrConfigurationTopic");
    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader(sub, topic);

    // Creation og the environment. Pointer to Environment can now be passed to service, transparently of the actual implementation.
    kpsr::dds_mdlw::DDSEnv envSub("./example2.yaml", "example2_conf", &datawriter, &datareader);
@endcode
 *
 */
class DDSEnv : public Environment
{
public:
    /**
     * @brief DDSEnv
     * @param yamlFileName file name to persist to. If empty, no persistent service.
     * @param ddsKey key to identify environment changes messages.
     * @param dataWriter
     * @param dataReader
     */
    DDSEnv(const std::string yamlFileName, std::string ddsKey,
           dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
           dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader);

    /**
     * @brief DDSEnv
     * @param yamlEnvironment
     * @param dataWriter
     * @param dataReader
     */
    DDSEnv(YamlEnvironment * yamlEnvironment,
           dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
           dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader);

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
private:
    void publishConfiguration();

    YamlEnvironment * _decorableEnv;

    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * _dataWriter;
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * _dataReader;
    DDSConfigurationListener * _listener;
    std::string _ddsKey;
    long _timestamp;

};
}
}
#endif
