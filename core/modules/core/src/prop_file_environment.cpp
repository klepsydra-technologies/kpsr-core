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

#include <klepsydra/core/prop_file_environment.h>

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(const std::string &propertyFileName)
    : _configFile(propertyFileName)
{}

kpsr::PropertyFileEnvironment::PropertyFileEnvironment(std::istream &propertyFileContent)
    : _configFile(propertyFileContent)
{}

void kpsr::PropertyFileEnvironment::getPropertyString(const std::string &key,
                                                      std::string &value,
                                                      std::string const &rootNode)
{
    value = _configFile.getValueOfKey<std::string>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyInt(const std::string &key,
                                                   int &value,
                                                   std::string const &rootNode)
{
    value = _configFile.getValueOfKey<int>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyFloat(const std::string &key,
                                                     float &value,
                                                     std::string const &rootNode)
{
    value = _configFile.getValueOfKey<float>(key);
}

void kpsr::PropertyFileEnvironment::getPropertyBool(const std::string &key,
                                                    bool &value,
                                                    std::string const &rootNode)
{
    value = _configFile.getValueOfKey<bool>(key);
}

void kpsr::PropertyFileEnvironment::setPropertyString(const std::string &key,
                                                      const std::string &value,
                                                      std::string const &rootNode)
{
    // Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyInt(const std::string &key,
                                                   const int &value,
                                                   std::string const &rootNode)
{
    // Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyFloat(const std::string &key,
                                                     const float &value,
                                                     std::string const &rootNode)
{
    // Unsupported
}

void kpsr::PropertyFileEnvironment::setPropertyBool(const std::string &key,
                                                    const bool &value,
                                                    std::string const &rootNode)
{
    // Unsupported
}

void kpsr::PropertyFileEnvironment::persist()
{
    // Unsupported
}

void kpsr::PropertyFileEnvironment::loadFile(const std::string &fileName,
                                             const std::string &nodeName)
{
    // Unsupported
}
