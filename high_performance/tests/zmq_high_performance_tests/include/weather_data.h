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

// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef WEATHER_DATA_H_
#define WEATHER_DATA_H_


// Include section.
#include <vector>
#include <memory>
#include <string>
#include <atomic>
#include "temperature.h"


// Klepsydra generated event class.
class WeatherData {
public:
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int constructorInvokations;
    static std::atomic_int copyInvokations;

    // Default constructor.
    WeatherData() {
        WeatherData::emptyConstructorInvokations++;
    }

    // Main constructor.
    WeatherData(
            std::string zipcode,
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
    WeatherData(const WeatherData & that)
        : zipcode(that.zipcode)
        , currentTemp(that.currentTemp)
        , currentRelHumidity(that.currentRelHumidity)
        , historicTemp(that.historicTemp)
        , historicRelHumidity(that.historicRelHumidity)
    {
        WeatherData::copyInvokations++;
    }

    // Clone method. Needed by klepsydra core APIs.
    void clone(const WeatherData & that) {
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
