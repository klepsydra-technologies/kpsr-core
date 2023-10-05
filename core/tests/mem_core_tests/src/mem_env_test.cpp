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

#include <klepsydra/mem_core/mem_env.h>

#include <spdlog/spdlog.h>

TEST(MemEnvironment, NominalTest)
{
    kpsr::mem::MemEnv environment;

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

    kpsr::mem::MemEnv newEnvironment;
    ASSERT_FALSE(
        newEnvironment.loadFile("dummy text which fail since mem env does not have load", ""));
}

TEST(MemEnvironment, NominalMissingWithDefaultsTest)
{
    kpsr::mem::MemEnv environment;

    std::string stringPropertyName = "testPropString";
    std::string stringPropertyValue = "dummyValue";

    std::string checkStringPropertyValue;
    ASSERT_FALSE(environment.getPropertyString(stringPropertyName, checkStringPropertyValue));
    ASSERT_EQ(checkStringPropertyValue, "");
    ASSERT_FALSE(environment.getPropertyString(stringPropertyName,
                                               checkStringPropertyValue,
                                               stringPropertyValue));
    ASSERT_EQ(checkStringPropertyValue, stringPropertyValue);

    std::string intPropertyName = "testPropInt";
    int intPropertyValue = 12;

    int checkIntPropertyValue = -1;
    ASSERT_FALSE(environment.getPropertyInt(intPropertyName, checkIntPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, 0);
    ASSERT_FALSE(
        environment.getPropertyInt(intPropertyName, checkIntPropertyValue, intPropertyValue));
    ASSERT_EQ(checkIntPropertyValue, intPropertyValue);

    std::string floatPropertyName = "testPropFloat";
    float floatPropertyValue = 3.145f;

    float checkFloatPropertyValue = -21412.1f;
    ASSERT_FALSE(environment.getPropertyFloat(floatPropertyName, checkFloatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, 0.0f);
    ASSERT_FALSE(environment.getPropertyFloat(floatPropertyName,
                                              checkFloatPropertyValue,
                                              floatPropertyValue));
    ASSERT_EQ(checkFloatPropertyValue, floatPropertyValue);

    std::string boolPropertyName = "testPropBool";
    bool boolPropertyValue = true;

    bool checkBoolPropertyValue(true);
    ASSERT_FALSE(environment.getPropertyBool(boolPropertyName, checkBoolPropertyValue));
    ASSERT_NE(checkBoolPropertyValue, boolPropertyValue);
}
