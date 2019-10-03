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

#include <klepsydra/core/prop_file_environment.h>

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
