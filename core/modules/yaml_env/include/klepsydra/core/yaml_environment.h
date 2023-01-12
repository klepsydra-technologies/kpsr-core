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

#ifndef YAML_ENVIRONMENT_H
#define YAML_ENVIRONMENT_H

#include <yaml-cpp/yaml.h>

#include <klepsydra/core/environment.h>

namespace kpsr {
/**
 * @brief The YamlEnvironment class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A YAML based implementation of the environment. Properties are read and written to the member YAML file.
 */
class YamlEnvironment : public Environment
{
public:
    /**
     * @brief YamlEnvironment
     * @param yamlFileName
     */
    YamlEnvironment(const std::string &yamlFileName,
                    const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief YamlEnvironment
     */
    YamlEnvironment();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string &key,
                                   std::string &value,
                                   const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string &key,
                                int &value,
                                const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string &key,
                                  float &value,
                                  const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string &key,
                                 bool &value,
                                 const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string &key,
                                   const std::string &value,
                                   const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string &key,
                                const int &value,
                                const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string &key,
                                  const float &value,
                                  const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string &key,
                                 const bool &value,
                                 const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * \brief reload
     * \param yamlContent
     */
    void updateConfiguration(const std::string &yamlContent);

    void updateConfiguration(const std::string &yamlContent, const std::string &rootNode);
    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

    virtual void loadFile(const std::string &fileName,
                          const std::string &nodeName = kpsr::DEFAULT_ROOT);

protected:
    const std::string _yamlFileName;
    YAML::Node _node;

private:
    YAML::Node getNode(const std::string &rootNode);
};
} // namespace kpsr

#endif // YAML_ENVIRONMENT_H
