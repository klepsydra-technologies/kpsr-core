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

#include "gtest/gtest.h"

#include <klepsydra/serialization/void_caster_mapper.h>

struct Event {
   int id;
   float fValue;
   double dValue;
};

TEST(SerializationTests, VoidCasterMapperTest) {
   std::vector<unsigned char> message;
   Event event;
   event.id = 1;
   event.fValue = 2.0;
   event.dValue = 3.0;

   kpsr::Mapper<Event, std::vector<unsigned char>> voidCasterMapper;
   voidCasterMapper.toMiddleware(event, message);

   Event fmEvent;
   voidCasterMapper.fromMiddleware(message, fmEvent);
   ASSERT_EQ(event.id, fmEvent.id);
   ASSERT_EQ(event.fValue, fmEvent.fValue);
   ASSERT_EQ(event.dValue, fmEvent.dValue);
}

