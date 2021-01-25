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

#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>
#include <klepsydra/dds_serialization/enum_dds_mapper.h>

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <klepsydra/dds_core/dds_env.h>
#include <klepsydra/dds_core/from_dds_middleware_provider.h>
#include <klepsydra/dds_core/to_dds_middleware_provider.h>

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
        , gotEnum(false), gotOctet(false), gotInt(false), gotBool(false), gotDouble(false), gotFloat(false), gotString(false)
        , _enumSubscriber(enumSubscriber)
        , _octetSubscriber(octetSubscriber)
        , _intSubscriber(intSubscriber)
        , _boolSubscriber(boolSubscriber)
        , _doubleSubscriber(doubleSubscriber)
        , _floatSubscriber(floatSubscriber)
        , _stringSubscriber(stringSubscriber) {
    }

    ~PrimitiveSubscriberService() {
        shutdown();
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

    void stop() {
        this->_enumSubscriber->removeListener("primitive_subscriber_service");
        this->_octetSubscriber->removeListener("primitive_subscriber_service");
        this->_intSubscriber->removeListener("primitive_subscriber_service");
        this->_boolSubscriber->removeListener("primitive_subscriber_service");
        this->_doubleSubscriber->removeListener("primitive_subscriber_service");
        this->_floatSubscriber->removeListener("primitive_subscriber_service");
        this->_stringSubscriber->removeListener("primitive_subscriber_service");

    }
    void execute() {}

    GreetEnum enumData;
    unsigned char charData;
    int intData;
    bool boolData;
    double doubleData;
    float floatData;
    std::string stringData;

    bool gotEnum, gotOctet, gotInt, gotBool, gotDouble, gotFloat, gotString;

private:

    void onEnumDataReceived(const GreetEnum & eventData) {
        enumData = eventData;
        gotEnum = true;
    }

    void onOctetDataReceived(const unsigned char & eventData) {
        charData = eventData;
        gotOctet = true;
    }

    void onIntDataReceived(const int & eventData) {
        intData = eventData;
        gotInt = true;
    }

    void onBoolDataReceived(const bool & eventData) {
        boolData = eventData;
        gotBool = true;
    }

    void onDoubleDataReceived(const double & eventData) {
        doubleData = eventData;
        gotDouble = true;
    }

    void onFloatDataReceived(const float & eventData) {
        floatData = eventData;
        gotFloat = true;
    }

    void onStringDataReceived(const std::string & eventData) {
        stringData = eventData;
        gotString = true;
    }

    kpsr::Subscriber<GreetEnum> * _enumSubscriber;
    kpsr::Subscriber<unsigned char> * _octetSubscriber;
    kpsr::Subscriber<int> * _intSubscriber;
    kpsr::Subscriber<bool> * _boolSubscriber;
    kpsr::Subscriber<double> * _doubleSubscriber;
    kpsr::Subscriber<float> * _floatSubscriber;
    kpsr::Subscriber<std::string> * _stringSubscriber;
};


class DDSCoreTest : public ::testing::Test {
protected:
    DDSCoreTest()
        : dp(0)
        , publisher(dp)
        , subscriber(dp)
        , enumDataTopic(dp, "enumData")
        , octetDataTopic(dp, "octetData")
        , longDataTopic(dp, "longData")
        , boolDataTopic(dp, "boolData")
        , doubleDataTopic(dp, "doubleData")
        , floatDataTopic(dp, "floatData")
        , stringDataTopic(dp, "stringData")
        , enumDataWriter(publisher, enumDataTopic)
        , octetDataWriter(publisher, octetDataTopic)
        , longDataWriter(publisher, longDataTopic)
        , boolDataWriter(publisher, boolDataTopic)
        , doubleDataWriter(publisher, doubleDataTopic)
        , floatDataWriter(publisher, floatDataTopic)
        , stringDataWriter(publisher, stringDataTopic)
        , enumDataReader(subscriber, enumDataTopic)
        , enumDataSafeQueueProvider(nullptr, "enumData", 0, nullptr, nullptr)
        , octetDataReader(subscriber, octetDataTopic)
        , octetDataSafeQueueProvider(nullptr, "octetData", 0, nullptr, nullptr)
        , longDataReader(subscriber, longDataTopic)
        , longDataSafeQueueProvider(nullptr, "longData", 0, nullptr, nullptr)
        , boolDataReader(subscriber, boolDataTopic)
        , boolDataSafeQueueProvider(nullptr, "boolData", 0, nullptr, nullptr)
        , doubleDataReader(subscriber, doubleDataTopic)
        , doubleDataSafeQueueProvider(nullptr, "doubleData", 0, nullptr, nullptr)
        , floatDataReader(subscriber, floatDataTopic)
        , floatDataSafeQueueProvider(nullptr, "floatData", 0, nullptr, nullptr)
        , stringDataReader(subscriber, stringDataTopic)
        , stringDataSafeQueueProvider(nullptr, "stringData", 0, nullptr, nullptr)
        , provider(nullptr)
        , primitivePublisherService(provider.getToMiddlewareChannel<GreetEnum, kpsr_dds_serialization::LongData>("enumData", 0, nullptr, &enumDataWriter),
                                    provider.getToMiddlewareChannel<unsigned char, kpsr_dds_serialization::OctetData>("octetData", 0, nullptr, &octetDataWriter),
                                    provider.getToMiddlewareChannel<int, kpsr_dds_serialization::LongData>("longData", 0, nullptr, &longDataWriter),
                                    provider.getToMiddlewareChannel<bool, kpsr_dds_serialization::BoolData>("boolData", 0, nullptr, &boolDataWriter),
                                    provider.getToMiddlewareChannel<double, kpsr_dds_serialization::DoubleData>("doubleData", 0, nullptr, &doubleDataWriter),
                                    provider.getToMiddlewareChannel<float, kpsr_dds_serialization::FloatData>("floatData", 0, nullptr, &floatDataWriter),
                                    provider.getToMiddlewareChannel<std::string, kpsr_dds_serialization::StringData>("stringData", 0, nullptr, &stringDataWriter))
        , primitiveSubscriberService(enumDataSafeQueueProvider.getSubscriber(),
                                     octetDataSafeQueueProvider.getSubscriber(),
                                     longDataSafeQueueProvider.getSubscriber(),
                                     boolDataSafeQueueProvider.getSubscriber(),
                                     doubleDataSafeQueueProvider.getSubscriber(),
                                     floatDataSafeQueueProvider.getSubscriber(),
                                     stringDataSafeQueueProvider.getSubscriber())
        {}

    void SetUp() override {
            ddsProvider.registerToTopic("enumData", &enumDataReader, true, enumDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("octetData", &octetDataReader, true, octetDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("longData", &longDataReader, true, longDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("boolData", &boolDataReader, true, boolDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("doubleData", &doubleDataReader, true, doubleDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("floatData", &floatDataReader, true, floatDataSafeQueueProvider.getPublisher());
            ddsProvider.registerToTopic("stringData", &stringDataReader, true, stringDataSafeQueueProvider.getPublisher());
    }

    void TearDown() override {
        ddsProvider.unregisterFromTopic("enumData", &enumDataReader);
        ddsProvider.unregisterFromTopic("octetData", &octetDataReader);
        ddsProvider.unregisterFromTopic("longData", &longDataReader);
        ddsProvider.unregisterFromTopic("boolData", &boolDataReader);
        ddsProvider.unregisterFromTopic("doubleData", &doubleDataReader);
        ddsProvider.unregisterFromTopic("floatData", &floatDataReader);
        ddsProvider.unregisterFromTopic("stringData", &stringDataReader);
    }

    dds::domain::DomainParticipant dp;
    dds::pub::Publisher publisher;
    dds::sub::Subscriber subscriber;

    dds::topic::Topic<kpsr_dds_serialization::LongData> enumDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::OctetData> octetDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::LongData> longDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::BoolData> boolDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::DoubleData> doubleDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::FloatData> floatDataTopic;
    dds::topic::Topic<kpsr_dds_serialization::StringData> stringDataTopic;

    dds::pub::DataWriter<kpsr_dds_serialization::LongData> enumDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::OctetData> octetDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::LongData> longDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::BoolData> boolDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::DoubleData> doubleDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::FloatData> floatDataWriter;
    dds::pub::DataWriter<kpsr_dds_serialization::StringData> stringDataWriter;

    dds::sub::DataReader<kpsr_dds_serialization::LongData> enumDataReader;
    kpsr::EventEmitterMiddlewareProvider<GreetEnum> enumDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::OctetData> octetDataReader;
    kpsr::EventEmitterMiddlewareProvider<unsigned char> octetDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::LongData> longDataReader;
    kpsr::EventEmitterMiddlewareProvider<int> longDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::BoolData> boolDataReader;
    kpsr::EventEmitterMiddlewareProvider<bool> boolDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::DoubleData> doubleDataReader;
    kpsr::EventEmitterMiddlewareProvider<double> doubleDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::FloatData> floatDataReader;
    kpsr::EventEmitterMiddlewareProvider<float> floatDataSafeQueueProvider;
    dds::sub::DataReader<kpsr_dds_serialization::StringData> stringDataReader;
    kpsr::EventEmitterMiddlewareProvider<std::string> stringDataSafeQueueProvider;

    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider;
    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;
    PrimitivePublisherService primitivePublisherService;
    PrimitiveSubscriberService primitiveSubscriberService;
};

TEST_F(DDSCoreTest, PubSubTest) {
    primitivePublisherService.startup();
    primitiveSubscriberService.startup();
    int const MAX_ATTEMPTS = 500;
    int const NUM_TESTS = 100;
    for (int i = 0; i < NUM_TESTS; i ++) {
        primitivePublisherService.runOnce();
        int attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotOctet) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_EQ((primitivePublisherService._seq + 1), primitiveSubscriberService.charData);
        primitiveSubscriberService.gotOctet = false;

        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotEnum) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_EQ((GreetEnum) (primitivePublisherService._seq % 4), primitiveSubscriberService.enumData);
        primitiveSubscriberService.gotEnum = false;

        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotInt) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_EQ(primitivePublisherService._seq, primitiveSubscriberService.intData);
        primitiveSubscriberService.gotInt = false;
        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotBool) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_EQ((primitivePublisherService._seq % 2 == 0), primitiveSubscriberService.boolData);
        primitiveSubscriberService.gotBool = false;

        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotFloat) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_FLOAT_EQ((primitivePublisherService._seq * 100.0), primitiveSubscriberService.floatData);
        primitiveSubscriberService.gotFloat = false;

        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotDouble) && (--attempts >= 0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_FLOAT_EQ((primitivePublisherService._seq * 10.0), primitiveSubscriberService.doubleData);
        primitiveSubscriberService.gotDouble = false;

        attempts = MAX_ATTEMPTS;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        } while ((!primitiveSubscriberService.gotString) && (--attempts >=0));
        ASSERT_GT(attempts, 0) << i;
        ASSERT_EQ((primitivePublisherService._greetings[primitivePublisherService._seq % 4]), primitiveSubscriberService.stringData) << i;
        primitiveSubscriberService.gotString = false;
    }
    primitivePublisherService.shutdown();
    primitiveSubscriberService.shutdown();
}
