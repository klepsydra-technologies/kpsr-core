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

#include <sstream>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <zmq.hpp>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include <klepsydra/serialization/json_cereal_mapper.h>
#include <klepsydra/serialization/binary_cereal_mapper.h>

#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>

#include "weather_data.h"
#include "weather_data_serializer.h"

#include "gtest/gtest.h"

//  Provide random number from 0..(num-1)
#define within(num) (int) ((float)((num) * random ()) / (RAND_MAX + 1.0))

class WeatherDataClient {
public:

    void onWeatherDataReceived(const WeatherData & weatherData) {
        _totalTemp += weatherData.currentTemp.value;
        _numSamples++;
        if (_numSamples == 100) {
            spdlog::info("Average temperature was {}F", (int) (_totalTemp / _numSamples));
        }
    }

    int _numSamples = 0;
private:
    long _totalTemp = 0;
};

TEST(ZMQMiddlewareTest, JsonSingleTopicBasicNoPool) {

    WeatherData::emptyConstructorInvokations = 0;
    WeatherData::constructorInvokations = 0;
    WeatherData::copyInvokations = 0;

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "Weather";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<WeatherData> * toZMQPublisher = toZMQMiddlewareProvider.getJsonToMiddlewareChannel<WeatherData>(topic, 0);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsonFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 0, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _jsonFromZMQProvider->start();

    _jsonFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener = std::bind(
                &WeatherDataClient::onWeatherDataReceived, &weatherDataClient, std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom ((unsigned) time (NULL));
    for (int i = 0; i < 100; i ++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within (50) + 10);
        historyRelHumidity.push_back(within (50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within (215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000", temperature, within (50) + 10,
                                historyTemperature, historyRelHumidity);
        //  Send message to all subscribers
        toZMQPublisher->publish(weatherData);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (weatherDataClient._numSamples < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    _jsonFromZMQProvider->unregisterFromTopic(topic);
    _jsonFromZMQProvider->stop();
    _safeQueueProvider.stop();
    _safeQueueProvider.getSubscriber()->removeListener("WeatherDataClient");

    ASSERT_EQ(weatherDataClient._numSamples, 100);

    spdlog::info("FINISHED!!!");

    ASSERT_EQ(WeatherData::emptyConstructorInvokations, 100);
    ASSERT_EQ(WeatherData::constructorInvokations, 100);
    ASSERT_EQ(WeatherData::copyInvokations, 0);
}


TEST(ZMQMiddlewareTest, JsonSingleTopicBasicWithPool) {

    WeatherData::emptyConstructorInvokations = 0;
    WeatherData::constructorInvokations = 0;
    WeatherData::copyInvokations = 0;

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "Weather";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<WeatherData> * toZMQPublisher =  toZMQMiddlewareProvider.getJsonToMiddlewareChannel<WeatherData>(topic, 0);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsonFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _jsonFromZMQProvider->start();

    _jsonFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener = std::bind(
                &WeatherDataClient::onWeatherDataReceived, &weatherDataClient, std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom ((unsigned) time (NULL));
    for (int i = 0; i < 100; i ++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within (50) + 10);
        historyRelHumidity.push_back(within (50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within (215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000", temperature, within (50) + 10,
                                historyTemperature, historyRelHumidity);
        //  Send message to all subscribers
        toZMQPublisher->publish(weatherData);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (weatherDataClient._numSamples < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    _jsonFromZMQProvider->unregisterFromTopic(topic);
    _jsonFromZMQProvider->stop();
    _safeQueueProvider.stop();
    _safeQueueProvider.getSubscriber()->removeListener("WeatherDataClient");

    ASSERT_EQ(weatherDataClient._numSamples, 100);

    spdlog::info("FINISHED!!!");

    ASSERT_EQ(WeatherData::emptyConstructorInvokations, 6);
    ASSERT_EQ(WeatherData::constructorInvokations, 100);
    ASSERT_EQ(WeatherData::copyInvokations, 0);
}

TEST(ZMQMiddlewareTest, BinarySingleTopicBasicNoPool) {

    WeatherData::emptyConstructorInvokations = 0;
    WeatherData::constructorInvokations = 0;
    WeatherData::copyInvokations = 0;

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "Weather";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<WeatherData> * toZMQPublisher = toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<Base> * _binaryFromZMQProvider = _fromZmqMiddlewareProvider.getBinaryFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 0, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _binaryFromZMQProvider->start();

    _binaryFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener = std::bind(
                &WeatherDataClient::onWeatherDataReceived, &weatherDataClient, std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom ((unsigned) time (NULL));
    for (int i = 0; i < 100; i ++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within (50) + 10);
        historyRelHumidity.push_back(within (50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within (215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000", temperature, within (50) + 10,
                                historyTemperature, historyRelHumidity);
        //  Send message to all subscribers
        toZMQPublisher->publish(weatherData);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (weatherDataClient._numSamples < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    _binaryFromZMQProvider->unregisterFromTopic(topic);
    _binaryFromZMQProvider->stop();
    _safeQueueProvider.stop();
    _safeQueueProvider.getSubscriber()->removeListener("WeatherDataClient");

    ASSERT_EQ(weatherDataClient._numSamples, 100);

    spdlog::info("FINISHED!!!");

    ASSERT_EQ(WeatherData::emptyConstructorInvokations, 100);
    ASSERT_EQ(WeatherData::constructorInvokations, 100);
    ASSERT_EQ(WeatherData::copyInvokations, 0);
}


TEST(ZMQMiddlewareTest, BinarySingleTopicBasicWithPool) {

    WeatherData::emptyConstructorInvokations = 0;
    WeatherData::constructorInvokations = 0;
    WeatherData::copyInvokations = 0;

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "Weather";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<WeatherData> * toZMQPublisher =  toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<Base> * _binaryFromZMQProvider = _fromZmqMiddlewareProvider.getBinaryFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _binaryFromZMQProvider->start();

    _binaryFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener = std::bind(
                &WeatherDataClient::onWeatherDataReceived, &weatherDataClient, std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom ((unsigned) time (NULL));
    for (int i = 0; i < 100; i ++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within (50) + 10);
        historyRelHumidity.push_back(within (50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(std::shared_ptr<Temperature>(new Temperature(within (215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within (215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000", temperature, within (50) + 10,
                                historyTemperature, historyRelHumidity);
        //  Send message to all subscribers
        toZMQPublisher->publish(weatherData);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (weatherDataClient._numSamples < 100) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    _binaryFromZMQProvider->unregisterFromTopic(topic);
    _binaryFromZMQProvider->stop();
    _safeQueueProvider.stop();
    _safeQueueProvider.getSubscriber()->removeListener("WeatherDataClient");

    ASSERT_EQ(weatherDataClient._numSamples, 100);

    spdlog::info("FINISHED!!!");

    ASSERT_EQ(WeatherData::emptyConstructorInvokations, 6);
    ASSERT_EQ(WeatherData::constructorInvokations, 100);
    ASSERT_EQ(WeatherData::copyInvokations, 0);
}
