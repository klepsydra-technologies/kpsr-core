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

#include "gtest/gtest.h"
#include <klepsydra/core/container.h>
#include <klepsydra/core/service.h>

class dummyService : public kpsr::Service
{
public:
    explicit dummyService(const std::string &name)
        : kpsr::Service(nullptr, name)
    {}

protected:
    virtual void execute() {}
    virtual void start() {}
    virtual void stop() {}
};

TEST(ServiceStatsTest, BasicTests)
{
    {
        ASSERT_NO_FATAL_FAILURE(kpsr::ServiceStats("dummyService"));
    }
    kpsr::ServiceStats dummyServiceStats("dummyService");

    ASSERT_EQ(0, dummyServiceStats.getTotalRunningTimeMs());

    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stopTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.startTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.startTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stopTimeWatch());
    ASSERT_LE(0, dummyServiceStats.getTotalRunningTimeMs());

    ASSERT_EQ(0, dummyServiceStats.getMillisecondsSinceStart());

    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stop());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.start());
    ASSERT_LE(0, dummyServiceStats.getMillisecondsSinceStart());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stop());
}
