// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <klepsydra/mem_core/mem_env.h>

bool kpsr::mem::MemEnv::getPropertyString(const std::string &key,
                                          std::string &value,
                                          std::string const &defaultValue,
                                          std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto it = stringDataMap.find(key);
    if (it == stringDataMap.end()) {
        value = defaultValue;
        return false;
    } else {
        value = it->second;
        return true;
    }
}

bool kpsr::mem::MemEnv::getPropertyInt(const std::string &key,
                                       int &value,
                                       const int defaultValue,
                                       std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto it = intDataMap.find(key);
    if (it == intDataMap.end()) {
        value = defaultValue;
        return false;
    } else {
        value = it->second;
        return true;
    }
}

bool kpsr::mem::MemEnv::getPropertyFloat(const std::string &key,
                                         float &value,
                                         float const defaultValue,
                                         std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto it = floatDataMap.find(key);
    if (it == floatDataMap.end()) {
        value = defaultValue;
        return false;
    } else {
        value = it->second;
        return true;
    }
}

bool kpsr::mem::MemEnv::getPropertyBool(const std::string &key,
                                        bool &value,
                                        bool const defaultValue,
                                        std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto it = boolDataMap.find(key);
    if (it == boolDataMap.end()) {
        value = defaultValue;
        return false;
    } else {
        value = it->second;
        return true;
    }
}

void kpsr::mem::MemEnv::setPropertyString(const std::string &key,
                                          const std::string &value,
                                          std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    stringDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyInt(const std::string &key,
                                       const int &value,
                                       const std::string &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    intDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyFloat(const std::string &key,
                                         const float &value,
                                         const std::string &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    floatDataMap[key] = value;
}

void kpsr::mem::MemEnv::setPropertyBool(const std::string &key,
                                        const bool &value,
                                        const std::string &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    boolDataMap[key] = value;
}

bool kpsr::mem::MemEnv::loadFile(const std::string &fileName, const std::string &nodeName)
{
    return false;
}
