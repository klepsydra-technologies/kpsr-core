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

#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include <klepsydra/core/configuration_environment.h>

#include "config.h"
#include <gtest/gtest.h>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

class ConfigEnvironmentLogger : public ::testing::Test
{
protected:
    void SetUp()
    {
        default_logger = spdlog::default_logger();
        auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
        logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
        logger->set_level(spdlog::level::debug);
        spdlog::register_logger(logger);
        spdlog::set_default_logger(logger);
    }

    void TearDown()
    {
        spdlog::set_default_logger(default_logger);
        spdlog::drop("my_logger");
    }

    std::shared_ptr<spdlog::logger> default_logger;
    std::shared_ptr<spdlog::logger> logger;
    std::stringstream programLogStream;
};

TEST(ConfigurationEnvironment, NominalTest)
{
    kpsr::ConfigurationEnvironment environment;

    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "dummyValue";

    ASSERT_NO_THROW(environment.setPropertyString(stringPropertyName, stringPropertyValue));

    std::string checkStringPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyString(stringPropertyName, checkStringPropertyValue));
    ASSERT_EQ(checkStringPropertyValue, stringPropertyValue);

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName, intPropertyValue));

    int checkIntPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName, checkIntPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, intPropertyValue);

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 3.145f;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName, floatPropertyValue));

    float checkFloatPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, floatPropertyValue);

    std::string floatPropertyNameTwo = "testPropFloatTwo";
    float floatPropertyValueTwo = 3.145353f;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyNameTwo, floatPropertyValueTwo));

    float checkFloatPropertyValueTwo;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyNameTwo, checkFloatPropertyValueTwo));
    ASSERT_EQ(checkFloatPropertyValueTwo, floatPropertyValueTwo);

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = false;

    ASSERT_NO_THROW(environment.setPropertyBool(boolPropertyName, boolPropertyValue));

    bool checkBoolPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue));
    ASSERT_EQ(checkBoolPropertyValue, boolPropertyValue);

    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    spdlog::info(serializedEnvironment);
}

TEST_F(ConfigEnvironmentLogger, NominalTestInt)
{
    kpsr::ConfigurationEnvironment environment;

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName, intPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName, checkIntPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, intPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string intPropertyName2 = "testPropInt2";
    int intPropertyValue2 = 1256;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName2, intPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName2, checkIntPropertyValue2));
    ASSERT_EQ(checkIntPropertyValue2, intPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string intPropertyName3 = "testPropInt3";
    int intPropertyValue3 = 456341;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName3, intPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName3, checkIntPropertyValue3));
    ASSERT_EQ(checkIntPropertyValue3, intPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());
    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    ASSERT_FALSE(serializedEnvironment.empty());
    default_logger->info("Serialized environment: {}", serializedEnvironment);

    kpsr::ConfigurationEnvironment newEnvironment;
    ASSERT_NO_THROW(newEnvironment.updateConfiguration(serializedEnvironment));
    int checkIntPropertyValue3New;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName3, checkIntPropertyValue3New));
    ASSERT_EQ(checkIntPropertyValue3New, intPropertyValue3);
}

TEST_F(ConfigEnvironmentLogger, NominalTestFloat)
{
    kpsr::ConfigurationEnvironment environment;

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName, floatPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, floatPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string floatPropertyName2 = "testPropFloat2";
    float floatPropertyValue2 = 1256;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName2, floatPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName2, checkFloatPropertyValue2));
    ASSERT_EQ(checkFloatPropertyValue2, floatPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string floatPropertyName3 = "testPropFloat3";
    float floatPropertyValue3 = 456341;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName3, floatPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName3, checkFloatPropertyValue3));
    ASSERT_EQ(checkFloatPropertyValue3, floatPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());
    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    ASSERT_FALSE(serializedEnvironment.empty());
    default_logger->info("Serialized environment: {}", serializedEnvironment);

    kpsr::ConfigurationEnvironment newEnvironment;
    ASSERT_NO_THROW(newEnvironment.updateConfiguration(serializedEnvironment));
    float checkFloatPropertyValue3New;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName3, checkFloatPropertyValue3New));
    ASSERT_EQ(checkFloatPropertyValue3New, floatPropertyValue3);
}

TEST_F(ConfigEnvironmentLogger, NominalTestString)
{
    kpsr::ConfigurationEnvironment environment;

    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "12";

    ASSERT_NO_THROW(environment.setPropertyString(stringPropertyName, stringPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    std::string checkStringPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyString(stringPropertyName, checkStringPropertyValue));
    ASSERT_EQ(checkStringPropertyValue, stringPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string stringPropertyName2 = "testPropString2";
    std::string stringPropertyValue2 = "1256";

    ASSERT_NO_THROW(environment.setPropertyString(stringPropertyName2, stringPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    std::string checkStringPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyString(stringPropertyName2, checkStringPropertyValue2));
    ASSERT_EQ(checkStringPropertyValue2, stringPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string stringPropertyName3 = "testPropString3";
    std::string stringPropertyValue3 = "456341";

    ASSERT_NO_THROW(environment.setPropertyString(stringPropertyName3, stringPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    std::string checkStringPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyString(stringPropertyName3, checkStringPropertyValue3));
    ASSERT_EQ(checkStringPropertyValue3, stringPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    ASSERT_FALSE(serializedEnvironment.empty());
    default_logger->info("Serialized environment: {}", serializedEnvironment);

    kpsr::ConfigurationEnvironment newEnvironment;
    ASSERT_NO_THROW(newEnvironment.updateConfiguration(serializedEnvironment));
    std::string checkStringPropertyValue3New;
    ASSERT_NO_THROW(
        environment.getPropertyString(stringPropertyName3, checkStringPropertyValue3New));
    ASSERT_EQ(checkStringPropertyValue3New, stringPropertyValue3);
}

TEST_F(ConfigEnvironmentLogger, NominalTestBool)
{
    kpsr::ConfigurationEnvironment environment;

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyBool(boolPropertyName, boolPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    bool checkBoolPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue));
    ASSERT_EQ(checkBoolPropertyValue, boolPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string boolPropertyName2 = "testPropBool2";
    bool boolPropertyValue2 = 1256;

    ASSERT_NO_THROW(environment.setPropertyBool(boolPropertyName2, boolPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    bool checkBoolPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyBool(boolPropertyName2, checkBoolPropertyValue2));
    ASSERT_EQ(checkBoolPropertyValue2, boolPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string boolPropertyName3 = "testPropBool3";
    bool boolPropertyValue3 = 456341;

    ASSERT_NO_THROW(environment.setPropertyBool(boolPropertyName3, boolPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    bool checkBoolPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyBool(boolPropertyName3, checkBoolPropertyValue3));
    ASSERT_EQ(checkBoolPropertyValue3, boolPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    ASSERT_FALSE(serializedEnvironment.empty());
    default_logger->info("Serialized environment: {}", serializedEnvironment);

    kpsr::ConfigurationEnvironment newEnvironment;
    ASSERT_NO_THROW(newEnvironment.updateConfiguration(serializedEnvironment));
    bool checkBoolPropertyValue3New;
    ASSERT_NO_THROW(environment.getPropertyBool(boolPropertyName3, checkBoolPropertyValue3New));
    ASSERT_EQ(checkBoolPropertyValue3New, boolPropertyValue3);
}

TEST_F(ConfigEnvironmentLogger, NominalTestIntFloat)
{
    kpsr::ConfigurationEnvironment environment;

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName, intPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName, checkIntPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, intPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string intPropertyName2 = "testPropInt2";
    int intPropertyValue2 = 1256;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName2, intPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName2, checkIntPropertyValue2));
    ASSERT_EQ(checkIntPropertyValue2, intPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string intPropertyName3 = "testPropInt3";
    int intPropertyValue3 = 456341;

    ASSERT_NO_THROW(environment.setPropertyInt(intPropertyName3, intPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    int checkIntPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName3, checkIntPropertyValue3));
    ASSERT_EQ(checkIntPropertyValue3, intPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 12;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName, floatPropertyValue));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, floatPropertyValue);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string floatPropertyName2 = "testPropFloat2";
    float floatPropertyValue2 = 1256;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName2, floatPropertyValue2));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue2;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName2, checkFloatPropertyValue2));
    ASSERT_EQ(checkFloatPropertyValue2, floatPropertyValue2);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string floatPropertyName3 = "testPropFloat3";
    float floatPropertyValue3 = 456341;

    ASSERT_NO_THROW(environment.setPropertyFloat(floatPropertyName3, floatPropertyValue3));
    ASSERT_TRUE(programLogStream.str().empty());

    float checkFloatPropertyValue3;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName3, checkFloatPropertyValue3));
    ASSERT_EQ(checkFloatPropertyValue3, floatPropertyValue3);
    ASSERT_TRUE(programLogStream.str().empty());

    std::string serializedEnvironment;
    ASSERT_NO_THROW(serializedEnvironment = environment.exportEnvironment());
    ASSERT_FALSE(serializedEnvironment.empty());
    default_logger->info("Serialized environment: {}", serializedEnvironment);

    kpsr::ConfigurationEnvironment newEnvironment;
    ASSERT_NO_THROW(newEnvironment.updateConfiguration(serializedEnvironment));
    int checkIntPropertyValue3New;
    ASSERT_NO_THROW(environment.getPropertyInt(intPropertyName3, checkIntPropertyValue3New));
    ASSERT_EQ(checkIntPropertyValue3New, intPropertyValue3);

    float checkFloatPropertyValue3New;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyName3, checkFloatPropertyValue3New));
    ASSERT_EQ(checkFloatPropertyValue3New, floatPropertyValue3);
}

TEST_F(ConfigEnvironmentLogger, LoadFileTest)
{
    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_test.json";

    kpsr::ConfigurationEnvironment environment;
    ASSERT_NO_THROW(environment.loadFile(testFile, ""));

    std::string checkValue;
    ASSERT_NO_THROW(environment.getPropertyString("dummy", checkValue));
    ASSERT_FALSE(programLogStream.str().empty());

    std::string floatPropertyNameTwo = "testPropFloatTwo";
    float floatPropertyValueTwo = 3.145353f;

    float checkFloatPropertyValueTwo;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyNameTwo, checkFloatPropertyValueTwo));
    ASSERT_EQ(checkFloatPropertyValueTwo, floatPropertyValueTwo);
}

TEST_F(ConfigEnvironmentLogger, LoadFileMissingStringTest)
{
    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_missing_string.json";

    kpsr::ConfigurationEnvironment environment;
    ASSERT_NO_THROW(environment.loadFile(testFile, ""));

    ASSERT_NE(programLogStream.str().size(), 0);

    std::string floatPropertyNameTwo = "testPropFloatTwo";
    float floatPropertyValueTwo = 3.145353f;
    float checkFloatPropertyValueTwo(0);
    ASSERT_NO_THROW(environment.getPropertyFloat(floatPropertyNameTwo, checkFloatPropertyValueTwo));
    ASSERT_EQ(checkFloatPropertyValueTwo, floatPropertyValueTwo);
}

TEST_F(ConfigEnvironmentLogger, LoadFileMissingFloatTest)
{
    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_missing_float.json";

    kpsr::ConfigurationEnvironment environment;
    ASSERT_NO_THROW(environment.loadFile(testFile, ""));
    ASSERT_NE(programLogStream.str().size(), 0);
}

TEST_F(ConfigEnvironmentLogger, LoadFileMissingIntTest)
{
    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_missing_int.json";

    kpsr::ConfigurationEnvironment environment;
    ASSERT_NO_THROW(environment.loadFile(testFile, ""));
    ASSERT_NE(programLogStream.str().size(), 0);
}

TEST_F(ConfigEnvironmentLogger, LoadFileMissingBoolTest)
{
    std::string folderName(TEST_DATA);
    std::string testFile = folderName + "/config_env_missing_bool.json";

    kpsr::ConfigurationEnvironment environment;
    ASSERT_NO_THROW(environment.loadFile(testFile, ""));

    ASSERT_NE(programLogStream.str().size(), 0);
}
