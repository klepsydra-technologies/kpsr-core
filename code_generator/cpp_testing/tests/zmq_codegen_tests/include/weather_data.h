/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
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
