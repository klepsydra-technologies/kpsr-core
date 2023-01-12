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

#ifndef WEATHER_DATA_H_
#define WEATHER_DATA_H_

// Include section.
#include "temperature.h"
#include <atomic>
#include <memory>
#include <string>
#include <vector>

// Klepsydra generated event class.
class WeatherData
{
public:
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int constructorInvokations;
    static std::atomic_int copyInvokations;

    // Default constructor.
    WeatherData() { WeatherData::emptyConstructorInvokations++; }

    // Main constructor.
    WeatherData(std::string zipcode,
                Temperature currentTemp,
                int currentRelHumidity,
                std::vector<std::shared_ptr<Temperature>> historicTemp,
                std::vector<int> historicRelHumidity)
        : zipcode(zipcode)
        , currentTemp(currentTemp)
        , currentRelHumidity(currentRelHumidity)
        , historicTemp(historicTemp)
        , historicRelHumidity(historicRelHumidity)
    {
        WeatherData::constructorInvokations++;
    }

    // Clone constructor. Needed by klepsydra core APIs.
    WeatherData(const WeatherData &that)
        : zipcode(that.zipcode)
        , currentTemp(that.currentTemp)
        , currentRelHumidity(that.currentRelHumidity)
        , historicTemp(that.historicTemp)
        , historicRelHumidity(that.historicRelHumidity)
    {
        WeatherData::copyInvokations++;
    }

    // Clone method. Needed by klepsydra core APIs.
    void clone(const WeatherData &that)
    {
        this->zipcode = that.zipcode;
        this->currentTemp = that.currentTemp;
        this->currentRelHumidity = that.currentRelHumidity;
        this->historicTemp = that.historicTemp;
        this->historicRelHumidity = that.historicRelHumidity;
    }

    // List of fields.
    std::string zipcode;
    Temperature currentTemp;
    int currentRelHumidity;
    std::vector<std::shared_ptr<Temperature>> historicTemp;
    std::vector<int> historicRelHumidity;
};

std::atomic_int WeatherData::constructorInvokations(0);
std::atomic_int WeatherData::emptyConstructorInvokations(0);
std::atomic_int WeatherData::copyInvokations(0);
#endif
