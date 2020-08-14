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

#ifndef YAML_ENVIRONMENT_H
#define YAML_ENVIRONMENT_H

#include <yaml-cpp/yaml.h>

#include <klepsydra/core/environment.h>

namespace kpsr {
/**
 * @brief The YamlEnvironment class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A YAML based implementation of the environment. Properties are read and written to the member YAML file.
 */
class YamlEnvironment : public Environment {
public:
    /**
     * @brief YamlEnvironment
     * @param yamlFileName
     */
    YamlEnvironment(const std::string & yamlFileName, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief YamlEnvironment
     */
    YamlEnvironment();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string & key, std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string & key, int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string & key, float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string & key, bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string & key, const int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * \brief reload
     * \param yamlContent
     */
    void updateConfiguration(const std::string & yamlContent);

    void updateConfiguration(const std::string & yamlContent, const std::string & rootNode);
    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

    virtual void loadFile(const std::string & fileName, const std::string & nodeName = kpsr::DEFAULT_ROOT);

protected:
    const std::string _yamlFileName;
    YAML::Node _node;

private:
    YAML::Node getNode(const std::string& rootNode);
};
}

#endif // YAML_ENVIRONMENT_H
