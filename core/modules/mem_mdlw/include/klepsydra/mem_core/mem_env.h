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

#ifndef MEM_ENV_H
#define MEM_ENV_H

#include <klepsydra/core/environment.h>

#include <map>
#include <string>
#include <mutex>

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
    void getPropertyString(const std::string & key, std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string & key, int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string & key, float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string & key, bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string & key, const std::string & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string & key, const int & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string & key, const float & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string & key, const bool & value, const std::string & rootNode = kpsr::DEFAULT_ROOT);

    void loadFile(const std::string & fileName, const std::string & nodeName) {};
private:

   	std::map<std::string, std::string> stringDataMap;
   	std::map<std::string, int> intDataMap;
   	std::map<std::string, float> floatDataMap;
   	std::map<std::string, bool> boolDataMap;

    std::mutex mutex;

};
}
}
#endif
