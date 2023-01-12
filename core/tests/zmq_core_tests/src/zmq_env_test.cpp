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
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>

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
    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::zmq_mdlw::ZMQEnv envTest(filename, "test", topic, 100, publisher, subscriber);
    std::string nameInFile;
    envTest.getPropertyString("filename", nameInFile);
    ASSERT_EQ(nameInFile, basename);

    std::string basename2("testfile2.yaml");
    std::string filename2 = folderName + "/" + basename2;
    envTest.loadFile(filename2, "file2");

    std::string nameFile2;
    envTest.getPropertyString("filename", nameFile2, "file2");
    ASSERT_EQ(nameFile2, basename2);
}

TEST_F(ZMQEnvTest, UpdateConfigurationTests)
{
    zmq::socket_t subscriber2(context, ZMQ_SUB);
    subscriber2.connect(clientUrl);
    subscriber2.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::zmq_mdlw::ZMQEnv envTest(filename, "test", topic, 1000, publisher, subscriber, "file1");

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    kpsr::YamlEnvironment yamlEnvPub;

    yamlEnvPub.updateConfiguration(
        "greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_zmq_env_key: "
        "\"test\"\nkpsr_zmq_env_topic_name: \"env_data\"\nkpsr_zmq_env_poll_period: 1000",
        kpsr::DEFAULT_ROOT);
    kpsr::zmq_mdlw::ZMQEnv zmqEnvTest(&yamlEnvPub, publisher, subscriber2, kpsr::DEFAULT_ROOT);
    std::string default_greeting;
    zmqEnvTest.getPropertyString("greeting", default_greeting);
    // Check that default values have been loaded correctly to zmqEnvTest
    ASSERT_EQ("hello", default_greeting);

    // Test that loading new file will trigger an update across all environments
    std::string basename2("testfile2.yaml");
    std::string filename2 = folderName + "/" + basename2;
    envTest.loadFile(filename2, "file2");
    std::string nameFile;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    zmqEnvTest.getPropertyString("filename", nameFile, "file2");
    ASSERT_EQ(nameFile, basename2);

    std::string nameInFile;
    zmqEnvTest.getPropertyString("filename", nameInFile, "file1");
    ASSERT_EQ(nameInFile, basename);
    std::string greeting;
    ASSERT_ANY_THROW(zmqEnvTest.getPropertyString("greeting", greeting));
}
