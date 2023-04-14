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

#ifndef CONFIGURATION_PROPERTIES_ENVIRONMENT_H
#define CONFIGURATION_PROPERTIES_ENVIRONMENT_H

#include <klepsydra/core/configuration_properties.h>
#include <klepsydra/core/environment.h>

#include <memory>

namespace kpsr {

/**
 * @brief The ConfigurationEnvironment class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A template based implementation of the environment. Properties are read and written with type checking..
 */
class ConfigurationEnvironment : public Environment
{
public:
    ConfigurationEnvironment();

    explicit ConfigurationEnvironment(const std::string &configFileName,
                                      const std::string &rootNode = kpsr::DEFAULT_ROOT);

    ~ConfigurationEnvironment();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string &key,
                                   std::string &value,
                                   const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string &key,
                                int &value,
                                const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string &key,
                                  float &value,
                                  const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string &key,
                                 bool &value,
                                 const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string &key,
                                   const std::string &value,
                                   const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string &key,
                                const int &value,
                                const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string &key,
                                  const float &value,
                                  const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string &key,
                                 const bool &value,
                                 const std::string &rootNode = kpsr::DEFAULT_ROOT) override;

    /**
     * @brief loadFile
     * @param fileName
     * @param nodeName
     *
     * This method is used to load additional configuration data from another file. It might be used in cases where
     * additional data may be loaded later.
     */
    virtual void loadFile(const std::string &fileName, const std::string &nodeName) override;

    void updateConfiguration(const std::string &jsonContent);

    void updateConfiguration(const std::string &jsonContent, const std::string &rootNode);

    std::string exportEnvironment();

private:
    void printGetError(bool result, const std::string &key) const;
    void printSetError(bool result, const std::string &key) const;

    template<class Archive>
    void serialize(Archive &archive)
    {
        try {
            archive(cereal::make_nvp("StringProperties", _stringProperties));
        } catch (const std::exception &e) {
            spdlog::debug("No string properties found");
        }
        try {
            archive(cereal::make_nvp("FloatProperties", _floatProperties));
        } catch (const std::exception &e) {
            spdlog::debug("No float properties found");
        }
        try {
            archive(cereal::make_nvp("IntProperties", _intProperties));
        } catch (const std::exception &e) {
            spdlog::debug("No int properties found");
        }
        try {
            archive(cereal::make_nvp("BoolProperties", _boolProperties));
        } catch (const std::exception &e) {
            spdlog::debug("No bool properties found");
        }
    }

    void copyFrom(const ConfigurationEnvironment &otherEnvironment, const std::string &rootNode);

    ConfigurationProperties<std::string> _stringProperties;
    ConfigurationProperties<float> _floatProperties;
    ConfigurationProperties<int> _intProperties;
    ConfigurationProperties<bool> _boolProperties;
};
} // namespace kpsr

#endif
