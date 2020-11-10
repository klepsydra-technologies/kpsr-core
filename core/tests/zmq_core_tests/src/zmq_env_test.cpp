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

#include <gtest/gtest.h>

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/zmq_env.h>

#include <klepsydra/serialization/binary_cereal_mapper.h>
#include <klepsydra/serialization/json_cereal_mapper.h>

#include "config.h"

TEST(ZMQEnvTest, EnvironmentTests) {
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "env_data";

    //  Prepare our context and publisher and subscriber.
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://cvMat-tests.ipc");

    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;

    kpsr::zmq_mdlw::ZMQEnv envTest(filename,
                                   "test",
                                   topic,
                                   100,
                                   publisher,
                                   subscriber);
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


TEST(ZMQEnvTest, UpdateConfigurationTests) {
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "env_data";

    //  Prepare our context and publisher and subscriber.
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://cvMat-tests.ipc");

    std::string clientUrl = "tcp://localhost:9001";
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    zmq::socket_t subscriber2 (context, ZMQ_SUB);
    subscriber2.connect(clientUrl);
    subscriber2.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::zmq_mdlw::ZMQEnv envTest(filename,
                                   "test",
                                   topic,
                                   1000,
                                   publisher,
                                   subscriber,
                                   "file1");

    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    kpsr::YamlEnvironment yamlEnvPub;

    yamlEnvPub.updateConfiguration(
        "greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_zmq_env_key: \"test\"\nkpsr_zmq_env_topic_name: \"env_data\"\nkpsr_zmq_env_poll_period: 1000",
        kpsr::DEFAULT_ROOT);
    kpsr::zmq_mdlw::ZMQEnv zmqEnvTest(&yamlEnvPub,
                                      publisher,
                                      subscriber2,
                                      kpsr::DEFAULT_ROOT);
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

