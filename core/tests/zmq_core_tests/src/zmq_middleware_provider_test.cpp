// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <getopt.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <zmq.hpp>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include <klepsydra/serialization/binary_cereal_mapper.h>
#include <klepsydra/serialization/json_cereal_mapper.h>

#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>

#include "weather_data.h"
#include "weather_data_serializer.h"

#include "gtest/gtest.h"

//  Provide random number from 0..(num-1)
#define within(num) (int) ((float) ((num) *random()) / (RAND_MAX + 1.0))

class WeatherDataClient
{
public:
    void onWeatherDataReceived(const WeatherData &weatherData)
    {
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

class ZMQMiddlewareTest : public ::testing::Test
{
protected:
    ZMQMiddlewareTest()
        : serverUrl("tcp://*:5556")
        , clientUrl("tcp://localhost:5556")
        , syncUrl("tcp://localhost:5557")
        , syncServiceUrl("tcp://*:5557")
        , topic("Weather")
        , context(1)
        , publisher(context, ZMQ_PUB)
        , subscriber(context, ZMQ_SUB)
        , syncclient(context, ZMQ_REQ)
        , syncservice(context, ZMQ_REP)
        , toZMQMiddlewareProvider(nullptr, publisher)
    {
        WeatherData::emptyConstructorInvokations = 0;
        WeatherData::constructorInvokations = 0;
        WeatherData::copyInvokations = 0;
        publisher.bind(serverUrl);
        publisher.bind("ipc://weather.ipc");

        //  Socket to talk to server
        spdlog::info("Collecting updates from weather server...\n");

        subscriber.connect(clientUrl);
        subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());
        // Set up publisher corresponding to each input.
        syncclient.connect(syncUrl);
        //  - send a synchronization request
        zmq::message_t message("", 1);
        syncservice.bind(syncServiceUrl);
        // Set up publisher corresponding to each input.
        syncclient.connect(syncUrl);
        //  - send a synchronization request
        syncclient.send(message);
        //  - wait for synchronization reply
        zmq::message_t recvMessage;
        syncservice.recv(recvMessage);
        syncservice.send(message);
        syncclient.recv(recvMessage);
    }

    std::string serverUrl;
    std::string clientUrl;
    std::string syncUrl;
    std::string syncServiceUrl;
    std::string topic;
    zmq::context_t context;
    zmq::socket_t publisher;
    zmq::socket_t subscriber;
    zmq::socket_t syncclient;
    zmq::socket_t syncservice;
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
};

TEST_F(ZMQMiddlewareTest, JsonSingleTopicBasicNoPool)
{
    kpsr::Publisher<WeatherData> *toZMQPublisher =
        toZMQMiddlewareProvider.getJsonToMiddlewareChannel<WeatherData>(topic, 0);

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqChannel<std::string> *_jsonFromZMQProvider =
        _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData>
        _safeQueueProvider(nullptr, "weatherData", 4, 0, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _jsonFromZMQProvider->start();

    _jsonFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener =
        std::bind(&WeatherDataClient::onWeatherDataReceived,
                  &weatherDataClient,
                  std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom((unsigned) time(NULL));
    for (int i = 0; i < 100; i++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within(50) + 10);
        historyRelHumidity.push_back(within(50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within(215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000",
                                temperature,
                                within(50) + 10,
                                historyTemperature,
                                historyRelHumidity);
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

TEST_F(ZMQMiddlewareTest, JsonSingleTopicBasicWithPool)
{
    kpsr::Publisher<WeatherData> *toZMQPublisher =
        toZMQMiddlewareProvider.getJsonToMiddlewareChannel<WeatherData>(topic, 0);

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqChannel<std::string> *_jsonFromZMQProvider =
        _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData>
        _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _jsonFromZMQProvider->start();

    _jsonFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener =
        std::bind(&WeatherDataClient::onWeatherDataReceived,
                  &weatherDataClient,
                  std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom((unsigned) time(NULL));
    for (int i = 0; i < 100; i++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within(50) + 10);
        historyRelHumidity.push_back(within(50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within(215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000",
                                temperature,
                                within(50) + 10,
                                historyTemperature,
                                historyRelHumidity);
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

TEST_F(ZMQMiddlewareTest, BinarySingleTopicBasicNoPool)
{
    kpsr::Publisher<WeatherData> *toZMQPublisher =
        toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqChannel<Base> *_binaryFromZMQProvider =
        _fromZmqMiddlewareProvider.getBinaryFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData>
        _safeQueueProvider(nullptr, "weatherData", 4, 0, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _binaryFromZMQProvider->start();

    _binaryFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener =
        std::bind(&WeatherDataClient::onWeatherDataReceived,
                  &weatherDataClient,
                  std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom((unsigned) time(NULL));
    for (int i = 0; i < 100; i++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within(50) + 10);
        historyRelHumidity.push_back(within(50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within(215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000",
                                temperature,
                                within(50) + 10,
                                historyTemperature,
                                historyRelHumidity);
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

TEST_F(ZMQMiddlewareTest, BinarySingleTopicBasicWithPool)
{
    kpsr::Publisher<WeatherData> *toZMQPublisher =
        toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqChannel<Base> *_binaryFromZMQProvider =
        _fromZmqMiddlewareProvider.getBinaryFromMiddlewareChannel<WeatherData>(subscriber, 10);
    kpsr::mem::BasicMiddlewareProvider<WeatherData>
        _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, false);
    _safeQueueProvider.start();
    _binaryFromZMQProvider->start();

    _binaryFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
    WeatherDataClient weatherDataClient;
    std::function<void(const WeatherData &)> listener =
        std::bind(&WeatherDataClient::onWeatherDataReceived,
                  &weatherDataClient,
                  std::placeholders::_1);
    _safeQueueProvider.getSubscriber()->registerListener("WeatherDataClient", listener);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //  Initialize random number generator
    srandom((unsigned) time(NULL));
    for (int i = 0; i < 100; i++) {
        std::vector<int> historyRelHumidity(0);
        historyRelHumidity.push_back(within(50) + 10);
        historyRelHumidity.push_back(within(50) + 10);

        std::vector<std::shared_ptr<Temperature>> historyTemperature(0);
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));
        historyTemperature.push_back(
            std::shared_ptr<Temperature>(new Temperature(within(215) - 80, Temperature::CELSIUS)));

        Temperature temperature(within(215) - 80, Temperature::CELSIUS);

        WeatherData weatherData("100000",
                                temperature,
                                within(50) + 10,
                                historyTemperature,
                                historyRelHumidity);
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
