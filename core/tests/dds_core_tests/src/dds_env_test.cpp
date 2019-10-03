/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
