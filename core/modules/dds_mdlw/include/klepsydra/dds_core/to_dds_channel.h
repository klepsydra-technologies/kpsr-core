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
                 const std::string & name,
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
