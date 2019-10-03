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
    YamlEnvironment(const std::string yamlFileName);

    /**
     * @brief YamlEnvironment
     */
    YamlEnvironment();

    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    virtual void getPropertyString(const std::string key, std::string & value);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    virtual void getPropertyInt(const std::string key, int & value);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    virtual void getPropertyFloat(const std::string key, float & value);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    virtual void getPropertyBool(const std::string key, bool & value);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    virtual void setPropertyString(const std::string key, const std::string value);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    virtual void setPropertyInt(const std::string key, const int & value);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    virtual void setPropertyFloat(const std::string key, const float & value);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    virtual void setPropertyBool(const std::string key, const bool & value);

    /**
     * @brief persist
     */
    void persist();

    /**
     * \brief reload
     * \param yamlContent
     */
    void reload(std::string yamlContent);

    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

protected:
    const std::string _yamlFileName;
    YAML::Node _node;
};
}

#endif // YAML_ENVIRONMENT_H
