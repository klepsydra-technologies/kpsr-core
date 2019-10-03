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

#ifndef MEM_ENV_H
#define MEM_ENV_H

#include <klepsydra/core/environment.h>

#include <map>
#include <string>

namespace kpsr
{
namespace mem
{
/**
 * @brief The MemEnv class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-mem-test
 *
 * @details Simple in-memory environment manager implemented with std::map instances.
 */
class MemEnv : public Environment
{
public:
    /**
     * @brief getPropertyString
     * @param key
     * @param value
     */
    void getPropertyString(const std::string key, std::string & value);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string key, int & value);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string key, float & value);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string key, bool & value);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string key, const std::string value);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string key, const int & value);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string key, const float & value);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string key, const bool & value);

    /**
     * @brief persist empty implementation
     */
    void persist() {}

private:

   	std::map<std::string, std::string> stringDataMap;
   	std::map<std::string, int> intDataMap;
   	std::map<std::string, float> floatDataMap;
   	std::map<std::string, bool> boolDataMap;

    pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

};
}
}
#endif
