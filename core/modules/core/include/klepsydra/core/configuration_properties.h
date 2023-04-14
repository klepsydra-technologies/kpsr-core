#ifndef CONFIGURATION_PROPERTIES_H
#define CONFIGURATION_PROPERTIES_H

#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <vector>

#include <spdlog/spdlog.h>

#include <cereal/types/map.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <klepsydra/core/property.h>

namespace kpsr {
inline std::string getFullKey(const std::string &key, const std::string &rootNode)
{
    return rootNode.empty() ? key : rootNode + "/" + key;
}

class DynamicConfigurationProperties
{
public:
    DynamicConfigurationProperties() {}

    virtual ~DynamicConfigurationProperties() = default;

    template<class T>
    bool getProperty(const std::string &propertyName, T &propertyValue) const
    {
        auto it = _properties.find(propertyName);
        if (it != _properties.end()) {
            if (it->second->isSameType(propertyValue)) {
                auto property = dynamic_cast<Property<T> *>(it->second.get());
                propertyValue = static_cast<T>(property->value);
                return true;
            } else {
                spdlog::error("Wrong property type. Stored property has type {}",
                              it->second->type.name());
                return false;
            }
        } else {
            spdlog::error("Property {} not found.", propertyName);
            return false;
        }
    }

    template<class T>
    bool setProperty(const std::string &propertyName, const T &propertyValue)
    {
        auto it = _properties.find(propertyName);
        if (it != _properties.end()) {
            if (!it->second->isSameType(propertyValue)) {
                spdlog::error("Attempting to overwrite property with different data type");
                return false;
            } else {
                spdlog::debug("Overwriting old data with new value");
            }
        }
        auto insertionResult = _properties.insert(
            std::make_pair(propertyName, std::make_shared<Property<T>>(propertyValue)));
        return insertionResult.second;
    }

    void copyFrom(const DynamicConfigurationProperties &other, const std::string &prefix = "")
    {
        for (auto &keyValPair : other._properties) {
            auto newKey = getFullKey(keyValPair.first, prefix);
            auto insertionResult = _properties.insert(std::make_pair(newKey, keyValPair.second));
            if (!insertionResult.second) {
                spdlog::error("Could not copy entry with key: {}", keyValPair.first);
            }
        }
    }

    template<class Archive>
    void serialize(Archive &archive)
    {
        archive(cereal::make_nvp("Properties", _properties));
    }

private:
    std::map<std::string, std::shared_ptr<TypedProperty>> _properties;
};

template<class T>
class ConfigurationProperties
{
public:
    ConfigurationProperties() {}

    virtual ~ConfigurationProperties() = default;

    bool getProperty(const std::string &propertyName, T &propertyValue) const
    {
        auto it = _properties.find(propertyName);
        if (it != _properties.end()) {
            propertyValue = it->second.value;
            return true;
        } else {
            spdlog::error("Property {} not found.", propertyName);
            return false;
        }
    }

    bool setProperty(const std::string &propertyName, const T &propertyValue)
    {
        auto insertionResult = _properties.insert(
            std::make_pair(propertyName, Property<T>(propertyValue)));
        if (!insertionResult.second) {
            insertionResult.first->second = Property<T>(propertyValue);
            spdlog::debug("Overwriting old data with new value");
        }
        return true;
    }

    void copyFrom(const ConfigurationProperties<T> &other, const std::string &prefix = "")
    {
        for (auto &keyValPair : other._properties) {
            auto newKey = getFullKey(keyValPair.first, prefix);
            auto insertionResult = _properties.insert(std::make_pair(newKey, keyValPair.second));
            if (!insertionResult.second) {
                spdlog::error("Could not copy entry with key: {}", keyValPair.first);
            }
        }
    }

    template<class Archive>
    void serialize(Archive &archive)
    {
        archive(_properties);
    }

private:
    std::map<std::string, Property<T>> _properties;
};
} // namespace kpsr

namespace cereal {
// Epilogue and Progolue functions set to empty since json doesn't use outer object
template<>
inline void epilogue(cereal::JSONInputArchive &, const kpsr::DynamicConfigurationProperties &)
{}
template<>
inline void prologue(cereal::JSONInputArchive &, const kpsr::DynamicConfigurationProperties &)
{}

template<>
inline void epilogue(cereal::JSONOutputArchive &, const kpsr::DynamicConfigurationProperties &)
{}
template<>
inline void prologue(cereal::JSONOutputArchive &, const kpsr::DynamicConfigurationProperties &)
{}

template<class T>
inline void epilogue(cereal::JSONInputArchive &, const kpsr::ConfigurationProperties<T> &)
{}
template<class T>
inline void prologue(cereal::JSONInputArchive &, const kpsr::ConfigurationProperties<T> &)
{}

template<class T>
inline void epilogue(cereal::JSONOutputArchive &, const kpsr::ConfigurationProperties<T> &)
{}
template<class T>
inline void prologue(cereal::JSONOutputArchive &, const kpsr::ConfigurationProperties<T> &)
{}
} // namespace cereal

#endif
