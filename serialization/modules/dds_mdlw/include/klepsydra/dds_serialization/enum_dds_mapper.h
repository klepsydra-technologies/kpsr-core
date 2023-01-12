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

#ifndef ENUM_DDS_MAPPER_H
#define ENUM_DDS_MAPPER_H

#include "long_data.hpp"

#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class E>
/**
 * @brief The Mapper<E, kpsr_dds_serialization::LongData> class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    void fromMiddleware(const kpsr_dds_serialization::LongData &message, E &event)
    {
        event = (E) message.data();
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const E &event, kpsr_dds_serialization::LongData &message)
    {
        message.data(event);
        message.id(_id++);
    }

private:
    int _id = 0;
};
} // namespace kpsr
#endif //ENUM_DDS_MAPPER_H
