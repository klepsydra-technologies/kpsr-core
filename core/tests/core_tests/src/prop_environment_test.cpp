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

#include "config.h"
#include <klepsydra/core/prop_file_environment.h>

#include "gtest/gtest.h"

TEST(PropertyFileEnvironment, BasicTest)
{
    std::stringstream stream;
    stream << "str.property=value\n"
           << "bool.property=1\n"
           << "int.property=1\n"
           << "canopen_bin_array_status_idx=0x2120\n"
           << "float.property=3.14\n";
    kpsr::PropertyFileEnvironment environment(stream);

    std::string strValue;
    environment.getPropertyString("str.property", strValue);
    ASSERT_EQ(strValue, "value");

    bool boolValue;
    environment.getPropertyBool("bool.property", boolValue);
    ASSERT_EQ(boolValue, true);

    int intValue;
    environment.getPropertyInt("int.property", intValue);
    ASSERT_EQ(intValue, 1);

    int hexValue;
    environment.getPropertyInt("canopen_bin_array_status_idx", hexValue);
    ASSERT_EQ(hexValue, 0x2120);

    float floatValue;
    environment.getPropertyFloat("float.property", floatValue);
    ASSERT_FLOAT_EQ(floatValue, 3.14f);
}

TEST(PropertyFileEnvironment, FileTest)
{
    std::string folderName(TEST_DATA);
    std::string basename("propenvtest.txt");
    std::string filename = folderName + "/" + basename;

    kpsr::PropertyFileEnvironment environment(filename);
    std::string strValue;
    environment.getPropertyString("str.property", strValue);
    ASSERT_EQ(strValue, "value");

    bool boolValue;
    environment.getPropertyBool("bool.property", boolValue);
    ASSERT_EQ(boolValue, true);

    int intValue;
    environment.getPropertyInt("int.property", intValue);
    ASSERT_EQ(intValue, 1);

    int hexValue;
    environment.getPropertyInt("canopen_bin_array_status_idx", hexValue);
    ASSERT_EQ(hexValue, 0x2120);

    float floatValue;
    environment.getPropertyFloat("float.property", floatValue);
    ASSERT_FLOAT_EQ(floatValue, 3.14f);
}

TEST(PropertyFileEnvironment, FileTestNoExist)
{
    std::string folderName(TEST_DATA);
    std::string basename("propenvtest.txt");
    std::string filename = folderName + "/" + basename;

    kpsr::PropertyFileEnvironment environment(filename);
    std::string strValue;
    environment.getPropertyString("RandomValue", strValue);
    ASSERT_EQ(strValue, "");
}
