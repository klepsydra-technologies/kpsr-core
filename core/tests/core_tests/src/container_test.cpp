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

#include "gtest/gtest.h"
#include <klepsydra/core/core_container.h>
#include <klepsydra/sdk/service.h>

class dummyService : public kpsr::Service
{
public:
    explicit dummyService(kpsr::Container *container, const std::string &name)
        : kpsr::Service(container, nullptr, name)
    {}

protected:
    virtual void execute() {}
    virtual void start() {}
    virtual void stop() {}
};

TEST(ContainerTest, ConstructorTest)
{
    ASSERT_NO_THROW(kpsr::CoreContainer testContainer(nullptr, "testContainer"));
    ASSERT_NO_THROW(kpsr::CoreContainer testContainer2(nullptr, ""));
}

TEST(ContainerTest, BasicTests)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");

    ASSERT_NO_FATAL_FAILURE(testContainer.start());
    ASSERT_NO_FATAL_FAILURE(testContainer.stop());

    dummyService testService(nullptr, "testService");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&testService));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
    // Remove already removed Service
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));

    kpsr::FunctionStats dummyFunctionStats("dummy");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyFunctionStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummyFunctionStats));

    kpsr::PublicationStats dummyPublicationStats("dummyPub", "pub");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyPublicationStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummyPublicationStats));

    kpsr::SubscriptionStats dummySubscriptionStats("dummySub", "sub", "dummyType");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummySubscriptionStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummySubscriptionStats));

    kpsr::ServiceStats dummyServiceStats("dummyService");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach(&dummyServiceStats));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&dummyServiceStats));
}

TEST(ContainerTest, BasicServiceTest)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    dummyService testService(&testContainer, "testService");

    ASSERT_NO_FATAL_FAILURE(testContainer.start());
    ASSERT_NO_FATAL_FAILURE(testContainer.stop());

    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
    // Remove already removed Service
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
}

TEST(ContainerTest, BasicServiceRunOnceTest)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    dummyService testService(&testContainer, "testService");

    ASSERT_NO_FATAL_FAILURE(testContainer.start());
    ASSERT_NO_FATAL_FAILURE(testService.runOnce());

    ASSERT_NO_FATAL_FAILURE(testContainer.stop());
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
}

TEST(ContainerTest, BasicServiceStartStopTest)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    dummyService testService(&testContainer, "testService");

    ASSERT_NO_FATAL_FAILURE(testContainer.start());
    ASSERT_NO_FATAL_FAILURE(testService.startup());

    ASSERT_NO_FATAL_FAILURE(testService.shutdown());
    ASSERT_NO_FATAL_FAILURE(testContainer.stop());
    ASSERT_NO_FATAL_FAILURE(testContainer.detach(&testService));
}

TEST(ContainerTest, NullptrChecks)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::Service *) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::Service *) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::FunctionStats *) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::FunctionStats *) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::PublicationStats *) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::PublicationStats *) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::SubscriptionStats *) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::SubscriptionStats *) nullptr));

    ASSERT_NO_FATAL_FAILURE(testContainer.attach((kpsr::ServiceStats *) nullptr));
    ASSERT_NO_FATAL_FAILURE(testContainer.detach((kpsr::ServiceStats *) nullptr));
}
