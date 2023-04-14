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
// See the License for the specific lang

#include <fstream>

#include <cereal/archives/json.hpp>

#include <klepsydra/core/configuration_environment.h>
#include <klepsydra/core/configuration_properties.h>

#include <spdlog/spdlog.h>

namespace kpsr {

ConfigurationEnvironment::ConfigurationEnvironment() {}

ConfigurationEnvironment::ConfigurationEnvironment(const std::string &configFileName,
                                                   const std::string &rootNode)
{
    ConfigurationEnvironment tempEnvironment;
    std::ifstream inputFileStream(configFileName, std::ios::binary);
    {
        ::cereal::JSONInputArchive iarchive(inputFileStream);
        tempEnvironment.serialize(iarchive);
    }
    copyFrom(tempEnvironment, rootNode);
}

ConfigurationEnvironment::~ConfigurationEnvironment() {}

void ConfigurationEnvironment::printGetError(bool result, const std::string &key) const
{
    if (!result) {
        spdlog::warn("Environment does not have the property {}. Value not loaded", key);
    }
}

void ConfigurationEnvironment::printSetError(bool result, const std::string &key) const
{
    if (!result) {
        spdlog::error("Error setting property {}. Value not updated", key);
    }
}

void ConfigurationEnvironment::getPropertyString(const std::string &key,
                                                 std::string &value,
                                                 std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _stringProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey);
}

void ConfigurationEnvironment::getPropertyInt(const std::string &key,
                                              int &value,
                                              std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _intProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey);
}

void ConfigurationEnvironment::getPropertyFloat(const std::string &key,
                                                float &value,
                                                std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _floatProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey);
}

void ConfigurationEnvironment::getPropertyBool(const std::string &key,
                                               bool &value,
                                               std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _boolProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey);
}

void ConfigurationEnvironment::setPropertyString(const std::string &key,
                                                 const std::string &value,
                                                 std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _stringProperties.setProperty(fullKey, value);
    printSetError(getResult, fullKey);
}

void ConfigurationEnvironment::setPropertyInt(const std::string &key,
                                              const int &value,
                                              std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _intProperties.setProperty(fullKey, value);
    printSetError(getResult, fullKey);
}

void ConfigurationEnvironment::setPropertyFloat(const std::string &key,
                                                const float &value,
                                                std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _floatProperties.setProperty(fullKey, value);
    printSetError(getResult, fullKey);
}

void ConfigurationEnvironment::setPropertyBool(const std::string &key,
                                               const bool &value,
                                               std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _boolProperties.setProperty(fullKey, value);
    printSetError(getResult, fullKey);
}

void ConfigurationEnvironment::loadFile(const std::string &fileName, const std::string &nodeName)
{
    std::ifstream inputFileStream(fileName, std::ios::binary);
    ConfigurationEnvironment newProperties;
    {
        ::cereal::JSONInputArchive iarchive(inputFileStream);
        newProperties.serialize(iarchive);
    }
    copyFrom(newProperties, nodeName);
    return;
}

void ConfigurationEnvironment::copyFrom(const ConfigurationEnvironment &otherEnvironment,
                                        const std::string &rootNode)
{
    _stringProperties.copyFrom(otherEnvironment._stringProperties, rootNode);
    _boolProperties.copyFrom(otherEnvironment._boolProperties, rootNode);
    _intProperties.copyFrom(otherEnvironment._intProperties, rootNode);
    _floatProperties.copyFrom(otherEnvironment._floatProperties, rootNode);
}

void ConfigurationEnvironment::updateConfiguration(const std::string &jsonContent)
{
    std::stringstream ss;
    ss << jsonContent;
    {
        ::cereal::JSONInputArchive iarchive(ss);
        serialize(iarchive);
    }
    return;
}

void ConfigurationEnvironment::updateConfiguration(const std::string &jsonContent,
                                                   const std::string &rootNode)
{
    std::stringstream ss;
    ss << jsonContent;
    ConfigurationEnvironment newProperties;
    {
        ::cereal::JSONInputArchive iarchive(ss);
        newProperties.serialize(iarchive);
    }
    copyFrom(newProperties, rootNode);
    return;
}

std::string ConfigurationEnvironment::exportEnvironment()
{
    std::stringstream ss;
    {
        ::cereal::JSONOutputArchive oarchive(ss);
        serialize(oarchive);
    }
    return ss.str();
}
} // namespace kpsr

CEREAL_REGISTER_TYPE_WITH_NAME(kpsr::Property<bool>, "bool")
CEREAL_REGISTER_TYPE_WITH_NAME(kpsr::Property<int>, "int")
CEREAL_REGISTER_TYPE_WITH_NAME(kpsr::Property<float>, "float")
CEREAL_REGISTER_TYPE_WITH_NAME(kpsr::Property<std::string>, "std::string")
CEREAL_REGISTER_POLYMORPHIC_RELATION(kpsr::TypedProperty, kpsr::Property<bool>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(kpsr::TypedProperty, kpsr::Property<int>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(kpsr::TypedProperty, kpsr::Property<float>)
CEREAL_REGISTER_POLYMORPHIC_RELATION(kpsr::TypedProperty, kpsr::Property<std::string>)
