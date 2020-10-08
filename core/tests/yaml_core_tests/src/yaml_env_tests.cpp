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

#include <string.h>
#include <chrono>
#include<algorithm>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "gtest/gtest.h"

#include <klepsydra/core/yaml_environment.h>
#include "config.h"

TEST(YamlEnvironmentTest, SingleFileNoRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::YamlEnvironment environment(filename);

    std::string nameInFile;

    ASSERT_NO_THROW(environment.getPropertyString("filename", nameInFile));

    ASSERT_EQ(nameInFile, basename);

    std::string nameWithRootName;
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT));
    
    ASSERT_EQ(nameWithRootName, basename);
}

TEST(YamlEnvironmentTest, SingleFileNonDefaultRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;

    std::string rootName("firstFile");
    kpsr::YamlEnvironment environment(filename, rootName);

    std::string nameInFile;

    ASSERT_ANY_THROW(environment.getPropertyString("filename", nameInFile));

    std::string nameWithRootName;
    environment.getPropertyString("filename", nameWithRootName, rootName);
    
    ASSERT_EQ(nameWithRootName, basename);
}


TEST(YamlEnvironmentTest, TwoFilesDefaultAndNonDefaultRootTest) {

    std::string basename1("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename1 = folderName + "/" + basename1;

    kpsr::YamlEnvironment environment(filename1);

    std::string nameInFile;

    environment.getPropertyString("filename", nameInFile);

    ASSERT_EQ(nameInFile, basename1);

    std::string nameWithRootName;
    environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT);
    
    ASSERT_EQ(nameWithRootName, basename1);

    std::string basename2("testfile2.yaml");
    std::string filename2 = folderName + "/" + basename2;
    std::string rootName2("secondFile");
    
    environment.loadFile(filename2, rootName2);

    std::string secondNameInFile;
    environment.getPropertyString("filename", secondNameInFile, rootName2);

    ASSERT_EQ(secondNameInFile, basename2);
}

TEST(YamlEnvironmentTest, NoRootSetValueTest) {

    kpsr::YamlEnvironment environment;

    std::string nameInFile="testFile";
    std::string intName = "intValue";
    int testValueInt = 45;

    std::string floatName = "floatValue";
    float testValueFloat = 45.0f;

    ASSERT_NO_THROW(environment.setPropertyString("filename", nameInFile));
    ASSERT_NO_THROW(environment.setPropertyInt(intName, testValueInt));
    ASSERT_NO_THROW(environment.setPropertyFloat(floatName, testValueFloat));

    std::string valueInNode;
    ASSERT_NO_THROW(environment.getPropertyString("filename", valueInNode));
    ASSERT_EQ(nameInFile, valueInNode);

    std::string nameWithRootName;
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT));
    ASSERT_EQ(nameWithRootName, nameInFile);

    int intValueInNode;
    ASSERT_NO_THROW(environment.getPropertyInt(intName, intValueInNode));
    ASSERT_EQ(testValueInt, intValueInNode);

    float floatValueInNode;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatName, floatValueInNode));
    ASSERT_EQ(testValueFloat, floatValueInNode);
}

TEST(YamlEnvironmentTest, SetValueTest) {
    kpsr::YamlEnvironment environment;

    std::string root = "testRoot";
    std::string nameInFile="testFile";
    std::string intName = "intValue";
    int testValueInt = 45;
    std::string floatName = "floatValue";
    float testValueFloat = 45.0f;
    ASSERT_NO_THROW(environment.setPropertyString("filename", nameInFile, root));
    ASSERT_NO_THROW(environment.setPropertyInt(intName, testValueInt, root));
    ASSERT_NO_THROW(environment.setPropertyFloat(floatName, testValueFloat, root));

    std::string valueInNode;
    ASSERT_ANY_THROW(environment.getPropertyString("filename", valueInNode));
    ASSERT_NO_THROW(environment.getPropertyString("filename", valueInNode, root));
    ASSERT_EQ(valueInNode, nameInFile);

    int intValueInNode;
    ASSERT_NO_THROW(environment.getPropertyInt(intName, intValueInNode, root));
    ASSERT_EQ(testValueInt, intValueInNode);

    float floatValueInNode;
    ASSERT_NO_THROW(environment.getPropertyFloat(floatName, floatValueInNode, root));
    ASSERT_EQ(testValueFloat, floatValueInNode);
}

TEST(YamlEnvironmentTest, LoadFileNoRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::YamlEnvironment environment;

    std::string nameInFile;

    ASSERT_ANY_THROW(environment.getPropertyString("filename", nameInFile));

    ASSERT_NO_THROW(environment.loadFile(filename));
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameInFile));
    ASSERT_EQ(nameInFile, basename);

    std::string nameWithRootName;
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT));
    
    ASSERT_EQ(nameWithRootName, basename);
}

TEST(YamlEnvironmentTest, UpdateConfigurationNoRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::YamlEnvironment fileEnvironment(filename);
    kpsr::YamlEnvironment environment;

    std::string nameInFile;
    ASSERT_ANY_THROW(environment.getPropertyString("filename", nameInFile));

    std::string fileEnvironmentString = fileEnvironment.exportEnvironment();
    
    ASSERT_NO_THROW(environment.updateConfiguration(fileEnvironmentString));
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameInFile));
    ASSERT_EQ(nameInFile, basename);

    std::string nameWithRootName;
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT));
    
    ASSERT_EQ(nameWithRootName, basename);
}

TEST(YamlEnvironmentTest, UpdateConfigurationRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::YamlEnvironment fileEnvironment(filename);
    kpsr::YamlEnvironment environment;
    std::string root = "root";
    std::string nameInFile;
    ASSERT_ANY_THROW(environment.getPropertyString("filename", nameInFile, root));

    std::string fileEnvironmentString = fileEnvironment.exportEnvironment();
    
    ASSERT_NO_THROW(environment.updateConfiguration(fileEnvironmentString, root));
    ASSERT_NO_THROW(environment.getPropertyString("filename", nameInFile, root));
    ASSERT_EQ(nameInFile, basename);

}

TEST(YamlEnvironmentTest, keyThrowTest) {
    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt> (programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;

    kpsr::YamlEnvironment environment(filename);

    std::string dummyString;
    float dummyFloat;
    int dummyInt;
    bool dummyBool;
    std::string randomKey("randomKey");

    ASSERT_THROW(environment.getPropertyString(randomKey, dummyString), YAML::Exception);
    ASSERT_THROW(environment.getPropertyFloat(randomKey, dummyFloat), YAML::Exception);
    ASSERT_THROW(environment.getPropertyInt(randomKey, dummyInt), YAML::Exception);
    ASSERT_THROW(environment.getPropertyBool(randomKey, dummyBool), YAML::Exception);
    std::string spdlogString = programLogStream.str();
    ASSERT_NE(spdlogString.size(), 0);
    ASSERT_NE(spdlogString.find(randomKey), std::string::npos);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}
