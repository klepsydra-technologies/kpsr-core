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

#ifndef TO_DDS_CHANNEL_H
#define TO_DDS_CHANNEL_H

#include "dds/dds.hpp"

#include <klepsydra/core/object_pool_publisher.h>

namespace kpsr {
namespace dds_mdlw {
template<class M>
/**
 * @brief The ToDDSChannel class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-internal
 *
 * @details Klepsydra Event to DDS provider. It only uses standard DDS API and it expects a DDS writer.
 *
 */
class ToDDSChannel : public ObjectPoolPublisher<M>
{
public:
    /**
     * @brief ToDDSChannel
     * @param container
     * @param name
     * @param poolSize
     * @param initializerFunction
     * @param ddsWriter
     */
    ToDDSChannel(Container *container,
                 const std::string &name,
                 int poolSize,
                 std::function<void(M &)> initializerFunction,
                 dds::pub::DataWriter<M> *ddsWriter)
        : ObjectPoolPublisher<M>(container, name, "DDS", poolSize, initializerFunction, nullptr)
        , _ddsWriter(ddsWriter)
    {}

protected:
    void internalPublish(std::shared_ptr<const M> event) override
    {
        _ddsWriter->write(*event.get());
    }

private:
    dds::pub::DataWriter<M> *_ddsWriter;
};
} // namespace dds_mdlw
} // namespace kpsr
#endif
