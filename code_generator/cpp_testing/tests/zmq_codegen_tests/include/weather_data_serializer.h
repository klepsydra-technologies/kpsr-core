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


// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef WEATHER_DATA_H
#define WEATHER_DATA_H

#include <cereal/cereal.hpp>

#include "temperature_serializer.h"
#include "weather_data.h"
#include <cereal/types/memory.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

namespace cereal {
template<class Archive>
void serialize(Archive &archive, WeatherData &event)
{
    archive(CEREAL_NVP(event.zipcode),
            CEREAL_NVP(event.currentTemp),
            CEREAL_NVP(event.currentRelHumidity),
            CEREAL_NVP(event.historicTemp),
            CEREAL_NVP(event.historicRelHumidity));
}
} // namespace cereal
#endif // WEATHER_DATA_H
