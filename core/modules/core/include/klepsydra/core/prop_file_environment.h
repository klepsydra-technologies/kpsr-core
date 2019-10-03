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

#ifndef PROP_FILE_ENVIRONMENT_H
#define PROP_FILE_ENVIRONMENT_H

#include <klepsydra/core/environment.h>
#include <klepsydra/core/config_file.h>

namespace kpsr {
/**
 * @brief The PropertyFileEnvironment class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details A PROP_FILE based implementation of the environment. Properties are read and written to the member PROP_FILE file.
 */
class PropertyFileEnvironment : public Environment {
public:
    /**
     * @brief PropertyFileEnvironment
     * @param propertyFileFileName
     */
    PropertyFileEnvironment(const std::string propertyFileName);

    /**
     * @brief PropertyFileEnvironment
     */
    PropertyFileEnvironment(std::istream & propertyFileContent);

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
     * \param propertyFileContent
     */
    void reload(std::string propertyFileContent);

    /**
     * @brief exportEnvironment
     * @return
     */
    std::string exportEnvironment();

protected:
	ConfigFile _configFile;
};
}

#endif // PROP_FILE_ENVIRONMENT_H
