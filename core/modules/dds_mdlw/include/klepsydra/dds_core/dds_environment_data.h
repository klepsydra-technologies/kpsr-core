/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DDS_ENVIRONMENT_DATA_H
#define DDS_ENVIRONMENT_DATA_H

#include <string>

namespace kpsr {
namespace dds_mdlw {
/**
 * @brief The DDSEnvironmentData struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-internal
 */
struct DDSEnvironmentData
{
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
} // namespace dds_mdlw
} // namespace kpsr

#endif // DDS_ENVIRONMENT_DATA_H
