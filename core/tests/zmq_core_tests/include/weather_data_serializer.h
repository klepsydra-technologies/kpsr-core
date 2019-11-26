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

// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef WEATHER_DATA_H
#define WEATHER_DATA_H

#include <cereal/cereal.hpp>

#include "weather_data.h"
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/memory.hpp>
#include "temperature_serializer.h"

namespace cereal {
template<class Archive>
void serialize(Archive & archive, WeatherData & event)
{
   archive(
      CEREAL_NVP(event.zipcode),
      CEREAL_NVP(event.currentTemp),
      CEREAL_NVP(event.currentRelHumidity),
      CEREAL_NVP(event.historicTemp),
      CEREAL_NVP(event.historicRelHumidity)
   );
}
}
#endif // WEATHER_DATA_H
