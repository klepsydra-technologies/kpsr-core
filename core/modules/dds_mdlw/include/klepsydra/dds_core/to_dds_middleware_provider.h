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

#ifndef TO_DDS_MIDDLEWARE_PROVIDER_H
#define TO_DDS_MIDDLEWARE_PROVIDER_H

#include <map>
#include <memory>

#include <dds/dds.hpp>

#include <klepsydra/core/to_middleware_channel.h>

#include <klepsydra/dds_core/to_dds_channel.h>

namespace kpsr {
namespace dds_mdlw {
/**
 * @brief The ToDDSMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-composition
 *
 * @details DDS Middleware provider. It concentrate the creation of publisher and subscriber for a particular topic.
 * It also has advance configuration for performance enhancements.  Its use
 * is very straightforward as the following example shows:
@code
    // Creation of the DDS writer
    dds::domain::DomainParticipant dp(0);
    dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");
    dds::pub::Publisher pub(dp);
    dds::pub::DataWriter<tutorial::TempSensorType> dw(pub, topic);

    // The main one-line needed one for the whole process.
    kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);

    // Retrieve the concrete publisher to DDS. Please note the optional object pool tuning params.
    kpsr::Publisher<TemperatureData> * temperatureToDDSChannel = provider.getToMiddlewareChannel<TemperatureData, tutorial::TempSensorType>("temperature", 0, nullptr, &dw);
@endcode
 *
 */
class ToDDSMiddlewareProvider
{
public:
    /**
     * @brief ToDDSMiddlewareProvider
     * @param container
     */
    ToDDSMiddlewareProvider(Container *container)
        : _container(container)
    {}

    /**
     * @brief getToMiddlewareChannel
     * @param topic
     * @param poolSize
     * @param initializerFunction
     * @param dataWriter
     * @return
     */
    template<class T, class M>
    Publisher<T> *getToMiddlewareChannel(std::string topic,
                                         int poolSize,
                                         std::function<void(M &)> initializerFunction,
                                         dds::pub::DataWriter<M> *dataWriter)
    {
        auto search = _publisherMap.find(topic);
        if (search != _publisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(
                internalPointer);
            return publisher.get();
        } else {
            ToDDSChannel<M> *toDDSChannel = new ToDDSChannel<M>(_container,
                                                                topic,
                                                                poolSize,
                                                                initializerFunction,
                                                                dataWriter);
            std::shared_ptr<Publisher<T>> publisher(
                new ToMiddlewareChannel<T, M>(_container, topic + "_dds", toDDSChannel));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(publisher);
            _publisherMap[topic] = internalPointer;
            return publisher.get();
        }
    }

private:
    Container *_container;
    std::map<std::string, std::shared_ptr<void>> _publisherMap;
};
} // namespace dds_mdlw
} // namespace kpsr

#endif // TO_DDS_MIDDLEWARE_PROVIDER_H
