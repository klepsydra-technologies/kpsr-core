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

#ifndef TO_DDS_CHANNEL_H
#define TO_DDS_CHANNEL_H

#include "dds/dds.hpp"

#include <klepsydra/core/object_pool_publisher.h>

namespace kpsr
{
namespace dds_mdlw
{
template<class M>
/**
 * @brief The ToDDSChannel class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    ToDDSChannel(Container * container,
                 const std::string name,
                 int poolSize,
                 std::function<void(M &)> initializerFunction,
                 dds::pub::DataWriter<M> * ddsWriter)
        : ObjectPoolPublisher<M>(container, name, "DDS", poolSize, initializerFunction, nullptr)
        , _ddsWriter(ddsWriter)
    {}

protected:
    void internalPublish(std::shared_ptr<const M> event) override {
        _ddsWriter->write(*event.get());
    }

private:
    dds::pub::DataWriter<M> * _ddsWriter;
};
}
}
#endif
