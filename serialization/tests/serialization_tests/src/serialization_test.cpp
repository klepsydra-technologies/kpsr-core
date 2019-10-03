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

