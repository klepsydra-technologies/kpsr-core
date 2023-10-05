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

#ifndef PROP_FILE_ENVIRONMENT_H
#define PROP_FILE_ENVIRONMENT_H

#include <klepsydra/core/config_file.h>
#include <klepsydra/sdk/environment.h>

namespace kpsr {
/**
 * @brief The PropertyFileEnvironment class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A PROP_FILE based implementation of the environment. Properties are read and written to the member PROP_FILE file.
 */
class PropertyFileEnvironment : public Environment
{
public:
    /**
     * @brief PropertyFileEnvironment
     * @param propertyFileName
     */
    explicit PropertyFileEnvironment(const std::string &propertyFileName);

    /**
     * @brief PropertyFileEnvironment
     */
    explicit PropertyFileEnvironment(std::istream &propertyFileContent);

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual bool getPropertyString(const std::string &key,
                                   std::string &value,
                                   const std::string &defaultValue = "",
                                   const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual bool getPropertyInt(const std::string &key,
                                int &value,
                                const int defaultValue = 0,
                                const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual bool getPropertyFloat(const std::string &key,
                                  float &value,
                                  const float defaultValue = 0.0f,
                                  const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual bool getPropertyBool(const std::string &key,
                                 bool &value,
                                 const bool defaultValue = false,
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
     * @brief persist
     */
    void persist();

    /**
     * \brief reload
     * \param propertyFileContent
     */
    void reload(std::string propertyFileContent, std::string const &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

    virtual bool loadFile(const std::string &fileName, const std::string &nodeName);

protected:
    ConfigFile _configFile;
};
} // namespace kpsr

#endif // PROP_FILE_ENVIRONMENT_H
