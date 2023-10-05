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

void ConfigurationEnvironment::printSetError(bool result, const std::string &key) const
{
    if (!result) {
        spdlog::error("Error setting property {}. Value not updated", key);
    }
}

bool ConfigurationEnvironment::getPropertyString(const std::string &key,
                                                 std::string &value,
                                                 std::string const &defaultValue,
                                                 std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _stringProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey, value, defaultValue);
    return getResult;
}

bool ConfigurationEnvironment::getPropertyInt(const std::string &key,
                                              int &value,
                                              const int defaultValue,
                                              std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _intProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey, value, defaultValue);
    return getResult;
}

bool ConfigurationEnvironment::getPropertyFloat(const std::string &key,
                                                float &value,
                                                const float defaultValue,
                                                std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _floatProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey, value, defaultValue);
    return getResult;
}

bool ConfigurationEnvironment::getPropertyBool(const std::string &key,
                                               bool &value,
                                               const bool defaultValue,
                                               std::string const &rootNode)
{
    auto fullKey = getFullKey(key, rootNode);
    auto getResult = _boolProperties.getProperty(fullKey, value);
    printGetError(getResult, fullKey, value, defaultValue);
    return getResult;
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

bool ConfigurationEnvironment::loadFile(const std::string &fileName, const std::string &nodeName)
{
    std::ifstream inputFileStream(fileName, std::ios::binary);
    ConfigurationEnvironment newProperties;
    try {
        ::cereal::JSONInputArchive iarchive(inputFileStream);
        newProperties.serialize(iarchive);
    } catch (::cereal::Exception &ex) {
        spdlog::error("{} Error loading Environment from file {}", ex.what(), fileName);
        return false;
    }

    return copyFrom(newProperties, nodeName);
}

bool ConfigurationEnvironment::copyFrom(const ConfigurationEnvironment &otherEnvironment,
                                        const std::string &rootNode)
{
    bool copyResult = true;
    copyResult = copyResult &&
                 _stringProperties.copyFrom(otherEnvironment._stringProperties, rootNode);
    copyResult = copyResult && _boolProperties.copyFrom(otherEnvironment._boolProperties, rootNode);
    copyResult = copyResult && _intProperties.copyFrom(otherEnvironment._intProperties, rootNode);
    copyResult = copyResult &&
                 _floatProperties.copyFrom(otherEnvironment._floatProperties, rootNode);
    return copyResult;
}

bool ConfigurationEnvironment::updateConfiguration(const std::string &jsonContent)
{
    std::stringstream ss;
    ss << jsonContent;
    try {
        ::cereal::JSONInputArchive iarchive(ss);
        serialize(iarchive);
    } catch (::cereal::Exception &ex) {
        spdlog::error("{} Error loading Environment from json string", ex.what());
        return false;
    }
    return true;
}

std::string ConfigurationEnvironment::exportEnvironment()
{
    std::stringstream ss;
    try {
        ::cereal::JSONOutputArchive oarchive(ss);
        serialize(oarchive);
    } catch (::cereal::Exception &ex) {
        spdlog::error("{} Error exporting Environment to json string", ex.what());
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
