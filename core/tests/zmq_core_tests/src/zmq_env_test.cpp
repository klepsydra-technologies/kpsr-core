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

#include <gtest/gtest.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/configuration_environment.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/service.h>
#include <klepsydra/sdk/subscriber.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/zmq_env.h>

#include <klepsydra/serialization/binary_cereal_mapper.h>
#include <klepsydra/serialization/json_cereal_mapper.h>

#include "config.h"

class ZMQEnvTest : public ::testing::Test
{
protected:
    ZMQEnvTest()
        : serverUrl("tcp://*:5556")
        , clientUrl("tcp://localhost:5556")
        , syncUrl("tcp://localhost:5557")
        , syncServiceUrl("tcp://*:5557")
        , topic("env_data")
        , context(1)
        , publisher(context, ZMQ_PUB)
        , subscriber(context, ZMQ_SUB)
        , syncclient(context, ZMQ_REQ)
        , syncservice(context, ZMQ_REP)
    {
        publisher.bind(serverUrl);
        publisher.bind("ipc://cvMat-tests.ipc");

        //  Socket to talk to server
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
};

TEST_F(ZMQEnvTest, EnvironmentTests)
{
    std::string basename("testfile1.json");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::zmq_mdlw::ZMQEnv envTest(filename, "test", topic, 100, publisher, subscriber);
    std::string nameInFile;
    ASSERT_TRUE(envTest.getPropertyString("filename", nameInFile));
    ASSERT_EQ(nameInFile, basename);

    std::string basename2("testfile2.json");
    std::string filename2 = folderName + "/" + basename2;
    ASSERT_TRUE(envTest.loadFile(filename2, "file2"));

    std::string nameFile2;
    ASSERT_TRUE(envTest.getPropertyString("filename", nameFile2, "", "file2"));
    ASSERT_EQ(nameFile2, basename2);
}

TEST_F(ZMQEnvTest, UpdateConfigurationTests)
{
    zmq::socket_t subscriber2(context, ZMQ_SUB);
    subscriber2.connect(clientUrl);
    subscriber2.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    std::string basename("testfile1.json");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::zmq_mdlw::ZMQEnv envTest(filename, "test", topic, 1000, publisher, subscriber, "file1");

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    kpsr::ConfigurationEnvironment confEnvPub;

    confEnvPub.updateConfiguration(
        R"({"StringProperties": {"greeting": "hello", "kpsr_zmq_env_key": "test", "kpsr_zmq_env_topic_name": "env_data"},)"
        R"( "IntProperties": {"test": 123, "iteration": -1, "kpsr_zmq_env_poll_period": 1000}})");

    kpsr::zmq_mdlw::ZMQEnv zmqEnvTest(&confEnvPub, publisher, subscriber2, kpsr::DEFAULT_ROOT);
    std::string default_greeting;
    ASSERT_TRUE(zmqEnvTest.getPropertyString("greeting", default_greeting));
    // Check that default values have been loaded correctly to zmqEnvTest
    ASSERT_EQ("hello", default_greeting);

    // Test that loading new file will trigger an update across all environments
    std::string basename2("testfile2.json");
    std::string filename2 = folderName + "/" + basename2;
    ASSERT_TRUE(envTest.loadFile(filename2, "file2"));
    std::string nameFile;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(zmqEnvTest.getPropertyString("filename", nameFile, "", "file2"));
    ASSERT_EQ(nameFile, basename2);

    std::string nameInFile;
    ASSERT_TRUE(zmqEnvTest.getPropertyString("filename", nameInFile, "", "file1"));
    ASSERT_EQ(nameInFile, basename);
    std::string greeting;
    ASSERT_FALSE(zmqEnvTest.getPropertyString("greeting", greeting));
    ASSERT_TRUE(greeting.empty());
}

TEST_F(ZMQEnvTest, NominalTest)
{
    kpsr::zmq_mdlw::ZMQEnv environment("", "test", topic, 100, publisher, subscriber);
    // kpsr::ConfigurationEnvironment environment;

    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "dummyValue";

    ASSERT_NO_THROW(environment.setPropertyString(stringPropertyName, stringPropertyValue));

    std::string checkStringPropertyValue;
    ASSERT_TRUE(environment.getPropertyString(stringPropertyName, checkStringPropertyValue));
    ASSERT_EQ(checkStringPropertyValue, stringPropertyValue);

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName, intPropertyValue));

    int checkIntPropertyValue;
    ASSERT_TRUE(environment.getPropertyInt(intPropertyName, checkIntPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, intPropertyValue);

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 3.145f;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName, floatPropertyValue));

    float checkFloatPropertyValue;
    ASSERT_TRUE(environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, floatPropertyValue);

    std::string floatPropertyNameTwo = "testPropFloatTwo";
    float floatPropertyValueTwo = 3.145353f;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyNameTwo, floatPropertyValueTwo));

    float checkFloatPropertyValueTwo;
    ASSERT_TRUE(environment.getPropertyFloat(floatPropertyNameTwo, checkFloatPropertyValueTwo));
    ASSERT_EQ(checkFloatPropertyValueTwo, floatPropertyValueTwo);

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = false;

    ASSERT_NO_THROW(environment.setPropertyBool(boolPropertyName, boolPropertyValue));

    bool checkBoolPropertyValue;
    ASSERT_TRUE(environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue));
    ASSERT_EQ(checkBoolPropertyValue, boolPropertyValue);

    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_test.json";

    // test file contains same keys as above. loading should fail unless root node is different
    ASSERT_FALSE(environment.loadFile(testFile, ""));
    ASSERT_TRUE(environment.loadFile(testFile, "file"));
    kpsr::ConfigurationEnvironment newEnvironment(testFile, "json");
    auto serializedData = newEnvironment.exportEnvironment();
    ASSERT_TRUE(environment.updateConfiguration(serializedData));
    ASSERT_FALSE(environment.updateConfiguration("random text that is not json"));
}
