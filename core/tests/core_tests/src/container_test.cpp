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

#include <klepsydra/core/container.h>
#include <klepsydra/core/service.h>
#include "gtest/gtest.h"

class dummyService : public kpsr::Service
{
public:
    explicit dummyService(const std::string& name) : kpsr::Service(nullptr, name) {}
protected:
    virtual void execute() {}
    virtual void start() {}
    virtual void stop() {}
};

TEST(ContainerTest, ConstructorTest) {

    ASSERT_NO_THROW(kpsr::Container testContainer(nullptr, "testContainer"));
    ASSERT_NO_THROW(kpsr::Container testContainer2(nullptr, ""));
}

TEST(ContainerTest, BasicTests) {
    kpsr::Container testContainer(nullptr, "testContainer");

    ASSERT_NO_FATAL_FAILURE(testContainer.start());
    ASSERT_NO_FATAL_FAILURE(testContainer.stop());

    dummyService testService("testService");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&testService));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
    // Remove already removed Service
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));

    kpsr::FunctionStats dummyFunctionStats("dummy");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyFunctionStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummyFunctionStats));

    kpsr::PublicationStats dummyPublicationStats("dummyPub", "pub");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyPublicationStats));

    kpsr::SubscriptionStats dummySubscriptionStats("dummySub", "sub", "dummyType");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummySubscriptionStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummySubscriptionStats));

    kpsr::ServiceStats dummyServiceStats("dummyService");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyServiceStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummyServiceStats));
}

TEST(ContainerTest, NullptrChecks) {
    kpsr::Container testContainer(nullptr, "testContainer");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::Service*) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::Service*) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::FunctionStats*) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::FunctionStats*) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::PublicationStats*) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::SubscriptionStats*) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::SubscriptionStats*) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::ServiceStats*) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::ServiceStats*) nullptr));
}
