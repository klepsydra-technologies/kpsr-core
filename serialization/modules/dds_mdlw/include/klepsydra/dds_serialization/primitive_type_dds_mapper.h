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

#ifndef PRIMITIVE_TYPE_DDS_MAPPER_H
#define PRIMITIVE_TYPE_DDS_MAPPER_H

#include <bool_data.hpp>
#include <double_data.hpp>
#include <float_data.hpp>
#include <long_data.hpp>
#include <long_long_data.hpp>
#include <octet_data.hpp>
#include <string_data.hpp>

#include <string>

#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<>
/**
 * @brief The Mapper<bool, kpsr_dds_serialization::BoolData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<bool, kpsr_dds_serialization::BoolData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::BoolData &message, bool &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const bool &event, kpsr_dds_serialization::BoolData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<unsigned char, kpsr_dds_serialization::OctetData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<unsigned char, kpsr_dds_serialization::OctetData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::OctetData &message, unsigned char &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const unsigned char &event, kpsr_dds_serialization::OctetData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<int, kpsr_dds_serialization::LongData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<int, kpsr_dds_serialization::LongData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::LongData &message, int &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const int &event, kpsr_dds_serialization::LongData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<long, kpsr_dds_serialization::LongLongData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<long, kpsr_dds_serialization::LongLongData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::LongLongData &message, long &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const long &event, kpsr_dds_serialization::LongLongData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<float, kpsr_dds_serialization::FloatData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<float, kpsr_dds_serialization::FloatData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::FloatData &message, float &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const float &event, kpsr_dds_serialization::FloatData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<double, kpsr_dds_serialization::DoubleData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<double, kpsr_dds_serialization::DoubleData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::DoubleData &message, double &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const double &event, kpsr_dds_serialization::DoubleData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};

template<>
/**
 * @brief The Mapper<std::string, kpsr_dds_serialization::StringData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<std::string, kpsr_dds_serialization::StringData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::StringData &message, std::string &event)
    {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const std::string &event, kpsr_dds_serialization::StringData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};
} // namespace kpsr

#endif // PRIMITIVE_TYPE_DDS_MAPPER_H
