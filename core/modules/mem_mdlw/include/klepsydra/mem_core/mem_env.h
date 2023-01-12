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

#ifndef MEM_ENV_H
#define MEM_ENV_H

#include <klepsydra/core/environment.h>

#include <map>
#include <mutex>
#include <string>

namespace kpsr {
namespace mem {
/**
 * @brief The MemEnv class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    void getPropertyString(const std::string &key,
                           std::string &value,
                           const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyInt
     * @param key
     * @param value
     */
    void getPropertyInt(const std::string &key,
                        int &value,
                        const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyFloat
     * @param key
     * @param value
     */
    void getPropertyFloat(const std::string &key,
                          float &value,
                          const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief getPropertyBool
     * @param key
     * @param value
     */
    void getPropertyBool(const std::string &key,
                         bool &value,
                         const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyString
     * @param key
     * @param value
     */
    void setPropertyString(const std::string &key,
                           const std::string &value,
                           const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyInt
     * @param key
     * @param value
     */
    void setPropertyInt(const std::string &key,
                        const int &value,
                        const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyFloat
     * @param key
     * @param value
     */
    void setPropertyFloat(const std::string &key,
                          const float &value,
                          const std::string &rootNode = kpsr::DEFAULT_ROOT);

    /**
     * @brief setPropertyBool
     * @param key
     * @param value
     */
    void setPropertyBool(const std::string &key,
                         const bool &value,
                         const std::string &rootNode = kpsr::DEFAULT_ROOT);

    void loadFile(const std::string &fileName, const std::string &nodeName){};

private:
    std::map<std::string, std::string> stringDataMap;
    std::map<std::string, int> intDataMap;
    std::map<std::string, float> floatDataMap;
    std::map<std::string, bool> boolDataMap;

    std::mutex mutex;
};
} // namespace mem
} // namespace kpsr
#endif
