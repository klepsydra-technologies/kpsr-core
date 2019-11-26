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

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <math.h>

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"

#include "spdlog/spdlog.h"
#include <spdlog/sinks/basic_file_sink.h>

#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>
#include <klepsydra/dds_serialization/enum_dds_mapper.h>

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>

#include <klepsydra/dds_core/from_dds_middleware_provider.h>
#include <klepsydra/dds_core/to_dds_middleware_provider.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

enum GreetEnum {
    HOLA = 0,
    HELLO,
    CIAO,
    HALLO
};

class PrimitivePublisherService : public kpsr::Service {
public:
    PrimitivePublisherService(kpsr::Publisher<GreetEnum> * enumPublisher,
                              kpsr::Publisher<unsigned char> * octetPublisher,
                              kpsr::Publisher<int> * intPublisher,
                              kpsr::Publisher<bool> * boolPublisher,
                              kpsr::Publisher<double> * doublePublisher,
                              kpsr::Publisher<float> * floatPublisher,
                              kpsr::Publisher<std::string> * stringPublisher)
        : kpsr::Service(nullptr, "primitive_publisher_ervice")
        , _enumPublisher(enumPublisher)
        , _octetPublisher(octetPublisher)
        , _intPublisher(intPublisher)
        , _boolPublisher(boolPublisher)
        , _doublePublisher(doublePublisher)
        , _floatPublisher(floatPublisher)
        , _stringPublisher(stringPublisher)
        , _greetings(4)
    {
        _greetings[0] = "HOLA";
        _greetings[1] = "HELLO";
        _greetings[2] = "CIAO";
        _greetings[3] = "HALLO";
    }

    void start() {}
    void stop() {}

    void execute() {
        spdlog::info("PrimitivePublisherService.runOnce");
        _seq++;
        GreetEnum greetEnum = (GreetEnum) (_seq % 4);
        _enumPublisher->publish(greetEnum);
        _octetPublisher->publish(_seq + 1);
        _intPublisher->publish(_seq);
        _boolPublisher->publish(_seq % 2 == 0);
        _doublePublisher->publish(_seq * 10.0);
        _floatPublisher->publish(_seq * 100.0);
        _stringPublisher->publish(_greetings[_seq %4]);
    }

    kpsr::Publisher<GreetEnum> * _enumPublisher;
    kpsr::Publisher<unsigned char> * _octetPublisher;
    kpsr::Publisher<int> * _intPublisher;
    kpsr::Publisher<bool> * _boolPublisher;
    kpsr::Publisher<double> * _doublePublisher;
    kpsr::Publisher<float> * _floatPublisher;
    kpsr::Publisher<std::string> * _stringPublisher;

    int _seq = 0;
    std::vector<std::string> _greetings;
};

class PrimitiveSubscriberService : public kpsr::Service {
public:
    PrimitiveSubscriberService(kpsr::Subscriber<GreetEnum> * enumSubscriber,
                               kpsr::Subscriber<unsigned char> * octetSubscriber,
                               kpsr::Subscriber<int> * intSubscriber,
                               kpsr::Subscriber<bool> * boolSubscriber,
                               kpsr::Subscriber<double> * doubleSubscriber,
                               kpsr::Subscriber<float> * floatSubscriber,
                               kpsr::Subscriber<std::string> * stringSubscriber)
        : Service(nullptr, "primitive_subscriber_service")
        , _enumSubscriber(enumSubscriber)
        , _octetSubscriber(octetSubscriber)
        , _intSubscriber(intSubscriber)
        , _boolSubscriber(boolSubscriber)
        , _doubleSubscriber(doubleSubscriber)
        , _floatSubscriber(floatSubscriber)
        , _stringSubscriber(stringSubscriber) {
    }

    void start() {
        std::function<void(GreetEnum)> enumListenerFunction = std::bind(&PrimitiveSubscriberService::onEnumDataReceived, this, std::placeholders::_1);
        this->_enumSubscriber->registerListener("primitive_subscriber_service", enumListenerFunction);

        std::function<void(unsigned char)> octetListenerFunction = std::bind(&PrimitiveSubscriberService::onOctetDataReceived, this, std::placeholders::_1);
        this->_octetSubscriber->registerListener("primitive_subscriber_service", octetListenerFunction);

        std::function<void(int)> intListenerFunction = std::bind(&PrimitiveSubscriberService::onIntDataReceived, this, std::placeholders::_1);
        this->_intSubscriber->registerListener("primitive_subscriber_service", intListenerFunction);

        std::function<void(bool)> boolListenerFunction = std::bind(&PrimitiveSubscriberService::onBoolDataReceived, this, std::placeholders::_1);
        this->_boolSubscriber->registerListener("primitive_subscriber_service", boolListenerFunction);

        std::function<void(double)> doubleListenerFunction = std::bind(&PrimitiveSubscriberService::onDoubleDataReceived, this, std::placeholders::_1);
        this->_doubleSubscriber->registerListener("primitive_subscriber_service", doubleListenerFunction);

        std::function<void(float)> floatListenerFunction = std::bind(&PrimitiveSubscriberService::onFloatDataReceived, this, std::placeholders::_1);
        this->_floatSubscriber->registerListener("primitive_subscriber_service", floatListenerFunction);

        std::function<void(std::string)> stringListenerFunction = std::bind(&PrimitiveSubscriberService::onStringDataReceived, this, std::placeholders::_1);
        this->_stringSubscriber->registerListener("primitive_subscriber_service", stringListenerFunction);
    }

    void stop() {}
    void execute() {}

    GreetEnum enumData;
    unsigned char charData;
    int intData;
    bool boolData;
    double doubleData;
    float floatData;
    std::string stringData;

private:

    void onEnumDataReceived(const GreetEnum & eventData) {
        enumData = eventData;
    }

    void onOctetDataReceived(const unsigned char & eventData) {
        charData = eventData;
    }

    void onIntDataReceived(const int & eventData) {
        intData = eventData;
    }

    void onBoolDataReceived(const bool & eventData) {
        boolData = eventData;
    }

    void onDoubleDataReceived(const double & eventData) {
        doubleData = eventData;
    }

    void onFloatDataReceived(const float & eventData) {
        floatData = eventData;
    }

    void onStringDataReceived(const std::string & eventData) {
        stringData = eventData;
    }

    kpsr::Subscriber<GreetEnum> * _enumSubscriber;
    kpsr::Subscriber<unsigned char> * _octetSubscriber;
    kpsr::Subscriber<int> * _intSubscriber;
    kpsr::Subscriber<bool> * _boolSubscriber;
    kpsr::Subscriber<double> * _doubleSubscriber;
    kpsr::Subscriber<float> * _floatSubscriber;
    kpsr::Subscriber<std::string> * _stringSubscriber;
};

TEST(DDSStgCoreTests, PubSubTest) {
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);

    dds::topic::Topic<kpsr_dds_serialization::LongData> enumDataTopic(dp, "enumData");
    dds::pub::DataWriter<kpsr_dds_serialization::LongData> enumDataWriter(publisher, enumDataTopic);
    kpsr::Publisher<GreetEnum> * enumPublisher =
            provider.getToMiddlewareChannel<GreetEnum, kpsr_dds_serialization::LongData>("enumData", 0, nullptr, &enumDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::OctetData> octetDataTopic(dp, "octetData");
    dds::pub::DataWriter<kpsr_dds_serialization::OctetData> octetDataWriter(publisher, octetDataTopic);
    kpsr::Publisher<unsigned char> * octetDataPublisher =
            provider.getToMiddlewareChannel<unsigned char, kpsr_dds_serialization::OctetData>("octetData", 0, nullptr, &octetDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::LongData> longDataTopic(dp, "longData");
    dds::pub::DataWriter<kpsr_dds_serialization::LongData> longDataWriter(publisher, longDataTopic);
    kpsr::Publisher<int> * longDataPublisher =
            provider.getToMiddlewareChannel<int, kpsr_dds_serialization::LongData>("longData", 0, nullptr, &longDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::BoolData> boolDataTopic(dp, "boolData");
    dds::pub::DataWriter<kpsr_dds_serialization::BoolData> boolDataWriter(publisher, boolDataTopic);
    kpsr::Publisher<bool> * boolDataPublisher =
            provider.getToMiddlewareChannel<bool, kpsr_dds_serialization::BoolData>("boolData", 0, nullptr, &boolDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::DoubleData> doubleDataTopic(dp, "doubleData");
    dds::pub::DataWriter<kpsr_dds_serialization::DoubleData> doubleDataWriter(publisher, doubleDataTopic);
    kpsr::Publisher<double> * doubleDataPublisher =
            provider.getToMiddlewareChannel<double, kpsr_dds_serialization::DoubleData>("doubleData", 0, nullptr, &doubleDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::FloatData> floatDataTopic(dp, "floatData");
    dds::pub::DataWriter<kpsr_dds_serialization::FloatData> floatDataWriter(publisher, floatDataTopic);
    kpsr::Publisher<float> * floatDataPublisher =
            provider.getToMiddlewareChannel<float, kpsr_dds_serialization::FloatData>("floatData", 0, nullptr, &floatDataWriter);

    dds::topic::Topic<kpsr_dds_serialization::StringData> stringDataTopic(dp, "stringData");
    dds::pub::DataWriter<kpsr_dds_serialization::StringData> stringDataWriter(publisher, stringDataTopic);
    kpsr::Publisher<std::string> * stringDataPublisher =
            provider.getToMiddlewareChannel<std::string, kpsr_dds_serialization::StringData>("stringData", 0, nullptr, &stringDataWriter);

    PrimitivePublisherService primitivePublisherService(enumPublisher,
                                                        octetDataPublisher,
                                                        longDataPublisher,
                                                        boolDataPublisher,
                                                        doubleDataPublisher,
                                                        floatDataPublisher,
                                                        stringDataPublisher);


    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;
    kpsr::high_performance::EventLoopMiddlewareProvider<128> eventloopProvider(nullptr);

    dds::sub::DataReader<kpsr_dds_serialization::LongData> enumDataReader(subscriber, enumDataTopic);
    ddsProvider.registerToTopic("enumData", &enumDataReader, true, eventloopProvider.getPublisher<GreetEnum>("enumData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::OctetData> octetDataReader(subscriber, octetDataTopic);
    ddsProvider.registerToTopic("octetData", &octetDataReader, true, eventloopProvider.getPublisher<unsigned char>("octetData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::LongData> longDataReader(subscriber, longDataTopic);
    ddsProvider.registerToTopic("longData", &longDataReader, true, eventloopProvider.getPublisher<int>("longData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::BoolData> boolDataReader(subscriber, boolDataTopic);
    ddsProvider.registerToTopic("boolData", &boolDataReader, true, eventloopProvider.getPublisher<bool>("boolData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::DoubleData> doubleDataReader(subscriber, doubleDataTopic);
    ddsProvider.registerToTopic("doubleData", &doubleDataReader, true, eventloopProvider.getPublisher<double>("doubleData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::FloatData> floatDataReader(subscriber, floatDataTopic);
    ddsProvider.registerToTopic("floatData", &floatDataReader, true, eventloopProvider.getPublisher<float>("floatData", 0, nullptr, nullptr));

    dds::sub::DataReader<kpsr_dds_serialization::StringData> stringDataReader(subscriber, stringDataTopic);
    ddsProvider.registerToTopic("stringData", &stringDataReader, true, eventloopProvider.getPublisher<std::string>("stringData", 0, nullptr, nullptr));

    PrimitiveSubscriberService primitiveSubscriberService(eventloopProvider.getSubscriber<GreetEnum>("enumData"),
                                                          eventloopProvider.getSubscriber<unsigned char>("octetData"),
                                                          eventloopProvider.getSubscriber<int>("longData"),
                                                          eventloopProvider.getSubscriber<bool>("boolData"),
                                                          eventloopProvider.getSubscriber<double>("doubleData"),
                                                          eventloopProvider.getSubscriber<float>("floatData"),
                                                          eventloopProvider.getSubscriber<std::string>("stringData"));
    eventloopProvider.start();
    primitivePublisherService.startup();
    primitiveSubscriberService.startup();

    for (int i = 0; i < 100; i ++) {
        primitivePublisherService.runOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ASSERT_EQ((GreetEnum) (primitivePublisherService._seq % 4), primitiveSubscriberService.enumData);
        ASSERT_EQ((primitivePublisherService._seq + 1), primitiveSubscriberService.charData);
        ASSERT_EQ((primitivePublisherService._seq), primitiveSubscriberService.intData);
        ASSERT_EQ((primitivePublisherService._seq % 2 == 0), primitiveSubscriberService.boolData);
        ASSERT_DOUBLE_EQ((primitivePublisherService._seq * 10.0), primitiveSubscriberService.doubleData);
        ASSERT_FLOAT_EQ((primitivePublisherService._seq * 100.0), primitiveSubscriberService.floatData);
        ASSERT_EQ((primitivePublisherService._greetings[primitivePublisherService._seq %4]), primitiveSubscriberService.stringData);
    }

    eventloopProvider.stop();

    ddsProvider.unregisterFromTopic("enumData", &enumDataReader);
    ddsProvider.unregisterFromTopic("octetData", &octetDataReader);
    ddsProvider.unregisterFromTopic("longData", &longDataReader);
    ddsProvider.unregisterFromTopic("boolData", &boolDataReader);
    ddsProvider.unregisterFromTopic("doubleData", &doubleDataReader);
    ddsProvider.unregisterFromTopic("floatData", &floatDataReader);
    ddsProvider.unregisterFromTopic("stringData", &stringDataReader);
}
