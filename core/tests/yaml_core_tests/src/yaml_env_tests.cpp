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

#include "gtest/gtest.h"

#include <klepsydra/core/yaml_environment.h>
#include "config.h"

TEST(YamlEnvironmentTest, SingleFileNoRootTest) {

    std::string basename("testfile1.yaml");
    std::string folderName(TEST_DATA);
    std::string filename = folderName + "/" + basename;
    kpsr::YamlEnvironment environment(filename);

    std::string nameInFile;

    environment.getPropertyString("filename", nameInFile);

    ASSERT_EQ(nameInFile, basename);

    std::string nameWithRootName;
    environment.getPropertyString("filename", nameWithRootName, kpsr::DEFAULT_ROOT);
    
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

