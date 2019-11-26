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
    yamlEnvSub.reload("greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_dds_env_key: \"test\"");
    kpsr::dds_mdlw::DDSEnv envSub(&yamlEnvSub, &datawriter1, &datareader1);

    kpsr::YamlEnvironment yamlEnvPub;
    yamlEnvPub.reload("greeting: \"hello\"\ntest: 123\niteration: -1\nkpsr_dds_env_key: \"test\"");
    kpsr::dds_mdlw::DDSEnv envPub(&yamlEnvPub, &datawriter2, &datareader2);

    envPub.setPropertyString("greeting", "hola");
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
