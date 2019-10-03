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

#ifndef DDS_ENVIRONMENT_DATA_H
#define DDS_ENVIRONMENT_DATA_H

#include <string>

namespace kpsr {
namespace dds_mdlw {
/**
 * @brief The DDSEnvironmentData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-internal
 */
struct DDSEnvironmentData {
public:
    /**
     * @brief configurationKey
     */
    std::string configurationKey;
    /**
     * @brief configurationData
     */
    std::string configurationData;
};
}
}

#endif // DDS_ENVIRONMENT_DATA_H
