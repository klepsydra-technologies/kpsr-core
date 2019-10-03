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

#ifndef ENUM_DDS_MAPPER_H
#define ENUM_DDS_MAPPER_H

#include "long_data.hpp"

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template<class E>
/**
 * @brief The Mapper<E, kpsr_dds_serialization::LongData> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-serialization
 *
 */
class Mapper<E, kpsr_dds_serialization::LongData>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const kpsr_dds_serialization::LongData& message, E& event) {
        event = (E) message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const E& event, kpsr_dds_serialization::LongData& message) {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};
}
#endif//ENUM_DDS_MAPPER_H
