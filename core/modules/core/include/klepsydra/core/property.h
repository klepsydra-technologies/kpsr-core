#ifndef PROPERTIES_H
#define PROPERTIES_H

#include <typeindex>

#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/archives/xml.hpp>

namespace kpsr {

class TypedProperty
{
public:
    TypedProperty()
        : type(std::type_index(typeid(int)))
    {}

    TypedProperty(const std::type_index &id)
        : type(id)
    {}

    virtual ~TypedProperty() {}

    template<class U>
    bool isSameType(const U &other)
    {
        return std::type_index(typeid(U)) == type;
    }
    template<class Archive>
    void serialize(Archive &archive)
    {}

    std::type_index type;
};

template<class T>
class Property : public TypedProperty
{
public:
    Property()
        : TypedProperty(typeid(T))
        , value()
    {}

    Property(const T &value_)
        : TypedProperty(typeid(T))
        , value(value_)
    {}

    template<class Archive>
    void serialize(Archive &archive)
    {
        archive(CEREAL_NVP(value));
    }
    template<class Archive>
    static void load_and_construct(Archive &ar, cereal::construct<Property<T>> &construct)
    {
        T x;
        ar(x);
        (void) construct(x);
    }

    T value;
};

} // namespace kpsr

namespace cereal {
//! Saving for std::map<std::string, KpsrClass> for text based archives
// Note that this shows off some internal cereal traits such as EnableIf,
// which will only allow this template to be instantiated if its predicates
// are true
template<class Archive,
         class C,
         class A,
         class KpsrClass,
         traits::EnableIf<traits::is_text_archive<Archive>::value> = traits::sfinae>
inline void save(Archive &ar, std::map<std::string, KpsrClass, C, A> const &map)
{
    for (const auto &i : map)
        ar(cereal::make_nvp(i.first, i.second));
}

//! Loading for std::map<std::string, std::vector<std::vector<float>> > for text based archives
template<class Archive,
         class C,
         class A,
         class KpsrClass,
         traits::EnableIf<traits::is_text_archive<Archive>::value> = traits::sfinae>
inline void load(Archive &ar, std::map<std::string, KpsrClass, C, A> &map)
{
    map.clear();

    auto hint = map.begin();
    while (true) {
        const auto namePtr = ar.getNodeName();

        if (!namePtr)
            break;

        std::string key = namePtr;
        KpsrClass value;
        ar(value);
        hint = map.emplace_hint(hint, std::move(key), std::move(value));
    }
}

template<class Archive,
         class C,
         class A,
         class KpsrClass,
         traits::EnableIf<traits::is_text_archive<Archive>::value> = traits::sfinae>
inline void save(Archive &ar, std::map<std::string, kpsr::Property<KpsrClass>, C, A> const &map)
{
    for (const auto &i : map)
        ar(cereal::make_nvp(i.first, i.second.value));
}

//! Loading for std::map<std::string, std::vector<std::vector<float>> > for text based archives
template<class Archive,
         class C,
         class A,
         class KpsrClass,
         traits::EnableIf<traits::is_text_archive<Archive>::value> = traits::sfinae>
inline void load(Archive &ar, std::map<std::string, kpsr::Property<KpsrClass>, C, A> &map)
{
    map.clear();

    auto hint = map.begin();
    while (true) {
        const auto namePtr = ar.getNodeName();

        if (!namePtr)
            break;

        std::string key = namePtr;
        KpsrClass value;
        ar(value);
        kpsr::Property<KpsrClass> propValue(value);
        hint = map.emplace_hint(hint, std::move(key), std::move(propValue));
    }
}
} // namespace cereal

#endif
