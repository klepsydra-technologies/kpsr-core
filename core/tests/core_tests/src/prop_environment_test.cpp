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

#include <klepsydra/core/prop_file_environment.h>
#include "config.h"

#include "gtest/gtest.h"

TEST(PropertyFileEnvironment, BasicTest) {
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

TEST(PropertyFileEnvironment, FileTest) {
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

TEST(PropertyFileEnvironment, FileTestNoExist) {
    std::string folderName(TEST_DATA);
    std::string basename("propenvtest.txt");
    std::string filename = folderName + "/" + basename;

    kpsr::PropertyFileEnvironment environment(filename);
    std::string strValue;
    environment.getPropertyString("RandomValue", strValue);
    ASSERT_EQ(strValue, "");
}
