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

void kpsr::mem::MemEnv::getPropertyString(const std::string &key,
                                          std::string &value,
                                          std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    value = stringDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyInt(const std::string &key,
                                       int &value,
                                       std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    value = intDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyFloat(const std::string &key,
                                         float &value,
                                         std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    value = floatDataMap[key];
}

void kpsr::mem::MemEnv::getPropertyBool(const std::string &key,
                                        bool &value,
                                        std::string const &rootNode)
{
    std::lock_guard<std::mutex> lock(mutex);
    value = boolDataMap[key];
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
