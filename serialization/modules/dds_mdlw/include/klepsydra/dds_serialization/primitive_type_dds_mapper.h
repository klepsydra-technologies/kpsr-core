/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#ifndef PRIMITIVE_TYPE_DDS_MAPPER_H
#define PRIMITIVE_TYPE_DDS_MAPPER_H

#include <bool_data.hpp>
#include <octet_data.hpp>
#include <long_data.hpp>
#include <long_long_data.hpp>
#include <double_data.hpp>
#include <float_data.hpp>
#include <string_data.hpp>

#include <string>

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template<>
/**
 * @brief The Mapper<bool, kpsr_dds_serialization::BoolData> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::BoolData& message, bool& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const bool& event, kpsr_dds_serialization::BoolData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::OctetData& message, unsigned char& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const unsigned char& event, kpsr_dds_serialization::OctetData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::LongData& message, int& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const int& event, kpsr_dds_serialization::LongData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::LongLongData& message, long& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const long& event, kpsr_dds_serialization::LongLongData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::FloatData& message, float& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const float& event, kpsr_dds_serialization::FloatData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::DoubleData& message, double& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const double& event, kpsr_dds_serialization::DoubleData& message) {
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
 * @copyright Klepsydra Technologies 2019-2020.
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
    void fromMiddleware(const kpsr_dds_serialization::StringData& message, std::string& event) {
        event = message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const std::string& event, kpsr_dds_serialization::StringData& message) {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};
}

#endif // PRIMITIVE_TYPE_DDS_MAPPER_H
