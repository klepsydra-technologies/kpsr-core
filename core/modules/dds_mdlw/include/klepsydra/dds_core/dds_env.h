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
    DDSConfigurationListener(const std::string & ddsKey, DDSEnv * ddsEnv, long sourceId)
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
     * @param yamlFileName file name to load configuration from.
     * @param ddsKey key to identify environment changes messages.
     * @param dataWriter
     * @param dataReader
     */
    DDSEnv(const std::string yamlFileName, std::string ddsKey,
           dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
           dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader,
           const std::string& rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief DDSEnv
     * @param yamlEnvironment
     * @param dataWriter
     * @param dataReader
     */
    DDSEnv(YamlEnvironment * yamlEnvironment,
           dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * dataWriter,
           dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * dataReader,
           const std::string& rootNode = kpsr::DEFAULT_ROOT);


    ~DDSEnv();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string & key, std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string & key, int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string & key, float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string & key, bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string & key, const int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT) override;

    virtual void loadFile(const std::string & fileName, const std::string & nodeName);

    /**
     * @brief updateConfiguration
     * @param configurationData
     */
    void updateConfiguration(const std::string & configurationData);

    /**
     * @brief updateConfiguration
     * @param configurationData
     * @param rootNode
     */
    void updateConfiguration(const std::string & configurationData, const std::string & rootNode);

private:
    void publishConfiguration();

    YamlEnvironment * _decorableEnv;

    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> * _dataWriter;
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> * _dataReader;
    DDSConfigurationListener * _listener;
    std::string _ddsKey;
    long _timestamp;
    bool _isEnvLocal;
};
}
}
#endif
