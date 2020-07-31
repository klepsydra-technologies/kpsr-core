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

#include <klepsydra/dds_core/dds_env.h>
#include "config.h"

TEST(DdsEnvironmentTest, DdsEnvironmentTest) {
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher pub(dp);
    dds::sub::Subscriber sub(dp);

    dds::topic::Topic<kpsr_dds_core::DDSEnvironmentData> topic(dp, "kpsrConfigurationTopic");
    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter1(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader1(sub, topic);

    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter2(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader2(sub, topic);

    kpsr::YamlEnvironment yamlEnvSub;
    yamlEnvSub.updateConfiguration("greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_dds_env_key: \"test\"");
    kpsr::dds_mdlw::DDSEnv envSub(&yamlEnvSub, &datawriter1, &datareader1);

    kpsr::YamlEnvironment yamlEnvPub;
    yamlEnvPub.updateConfiguration("greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_dds_env_key: \"test\"");
    kpsr::dds_mdlw::DDSEnv envPub(&yamlEnvPub, &datawriter2, &datareader2);

    envPub.setPropertyString("greeting", "hola");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::string greeting;
    int attempts = 200;
    do {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        envSub.getPropertyString("greeting", greeting);
    } while ((greeting.compare("hola")  != 0) && (--attempts >= 0));
    ASSERT_GT(attempts, 0);
    ASSERT_EQ(greeting, "hola");

    for (unsigned int i = 0; i < 100; ++i) {
        envPub.setPropertyInt("iteration", i);
        int iter;
        envSub.getPropertyInt("iteration", iter);
        attempts = 200;
        do {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            envSub.getPropertyInt("iteration", iter);
        } while ((iter != i) && (--attempts >= 0));
        ASSERT_GT(attempts, 0);
        ASSERT_EQ(iter, i);
    }
}

TEST(DdsEnvironmentTest, MultiYamlTestSingle) {
    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;

    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher pub(dp);
    dds::sub::Subscriber sub(dp);

    dds::topic::Topic<kpsr_dds_core::DDSEnvironmentData> topic(dp, "kpsrConfigurationTopic");

    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter1(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader1(sub, topic);
    kpsr::dds_mdlw::DDSEnv envSub(filename, "test", &datawriter1, &datareader1, "file1");

    std::string nameInFile;
    // get property should not work if rootnode "file1" is not provided.
    ASSERT_ANY_THROW(envSub.getPropertyString("filename", nameInFile));

    envSub.getPropertyString("filename", nameInFile, "file1");
    ASSERT_EQ(nameInFile, basename);
}

TEST(DdsEnvironmentTest, MultiYamlUpdateConfigurationTest) {
    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;

    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher pub(dp);
    dds::sub::Subscriber sub(dp);

    dds::topic::Topic<kpsr_dds_core::DDSEnvironmentData> topic(dp, "kpsrConfigurationTopic");

    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter1(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader1(sub, topic);
    kpsr::dds_mdlw::DDSEnv envSub(filename, "test", &datawriter1, &datareader1, "file1");

    // upload new file and see if updates other environments with same ddsKey
    // first set up new environment with same ddskey.
    dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter2(pub, topic);
    dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader2(sub, topic);
    kpsr::YamlEnvironment yamlEnvPub;
    yamlEnvPub.updateConfiguration("greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_dds_env_key: \"test\"", kpsr::DEFAULT_ROOT);
    kpsr::dds_mdlw::DDSEnv envPub(&yamlEnvPub, &datawriter2, &datareader2);

    std::string basename2("testfile2.yaml");
    std::string filename2 = folderName + "/" + basename2;
    envSub.loadFile(filename2, "file2");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::string nameFile2;
    ASSERT_NO_THROW(envPub.getPropertyString("filename", nameFile2, "file2"));
    ASSERT_EQ(nameFile2, basename2);

    std::string nameInFile;
    ASSERT_NO_THROW(envPub.getPropertyString("filename", nameInFile, "file1"));
    ASSERT_EQ(nameInFile, basename);
    std::string greeting;
    ASSERT_ANY_THROW(envPub.getPropertyString("greeting", greeting));
}
