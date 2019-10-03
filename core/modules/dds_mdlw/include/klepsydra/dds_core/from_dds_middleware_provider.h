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

#ifndef FROM_DDS_MIDDLEWARE_PROVIDER_H
#define FROM_DDS_MIDDLEWARE_PROVIDER_H

#include <algorithm>

#include "dds/dds.hpp"

#include <klepsydra/core/from_middleware_channel.h>

namespace kpsr
{
namespace dds_mdlw
{
template<class T, class M>
/**
 * @brief The DDSListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-internal
 */
class DDSListener : public dds::sub::NoOpDataReaderListener<M>
{
public:
    /**
     * @brief DDSListener
     * @param useTake
     * @param internalPublisher
     */
    DDSListener(bool useTake, Publisher<T> * internalPublisher, std::string topicName)
        : _useTake(useTake)
        , _fromMiddlewareChannel(internalPublisher)
        , _topicName(topicName)
    {}

    /**
     * @brief on_data_available
     * @param ddsReader
     */
    void on_data_available(dds::sub::DataReader<M> & ddsReader) {
        if (_useTake) {
            auto samples =  ddsReader.take();
            std::for_each(samples.begin(), samples.end(), [this](const rti::sub::LoanedSample<M>& s) {
                this->_fromMiddlewareChannel.onMiddlewareMessage(s.data());
            });
        }
        else {
            auto samples =  ddsReader.select().state(dds::sub::status::DataState::new_data()).read();
            std::for_each(samples.begin(), samples.end(), [this](const rti::sub::LoanedSample<M>& s) {
                this->_fromMiddlewareChannel.onMiddlewareMessage(s.data());
            });
        }
    }
private:
    bool _useTake;
    FromMiddlewareChannel<T, M> _fromMiddlewareChannel;
    std::string _topicName;
};


/**
 * @brief The FromDDSMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-dds-composition
 *
 *
 * @details This class listen to new data coming the DDS. It uses the standard DDS API. The use of this class is very
 * straightforward and just need a reference to a DDS Reader. The following example ilustrates this:
@code
    // DDS Reader creation example. Tuning of this are left to the developer.
    dds::domain::DomainParticipant dp(0);
    dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");
    dds::sub::Subscriber sub(dp);
    dds::sub::DataReader<tutorial::TempSensorType> dr(sub, topic);

    // Creation of the From DDS Provider. It is a one-liner only needed once for all readers.
    kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;

    // Create the publisher/subscriber pair that will be channeling the DDS events.
    kpsr::mem::SafeQueueMiddlewareProvider<TemperatureData> safeQueueProvider(nullptr, "example", 4, 0, nullptr, nullptr, false);
    safeQueueProvider.start();

    // Creating a concrete DDS reader and connect it to the publisher side
    ddsProvider.registerToTopic("TTempSensor", &dr, true, safeQueueProvider.getPublisher());

    // Finally, we pass the associated subscriber to the service that is going to use. Please note that this is
    // Transparent of the DDS completely, and yet extremely performing.
    TemperatureSubscriberService temperatureSubscriberService(nullptr, safeQueueProvider.getSubscriber());
@endcode
 *
 */
class FromDDSMiddlewareProvider
{
public:

    /**
     * @brief registerToTopic
     * @param topicName
     * @param ddsReader
     * @param useTake
     * @param internalPublisher
     */
    template<class T, class M>
    void registerToTopic(std::string topicName,
                         dds::sub::DataReader<M> * ddsReader,
                         bool useTake,
                         Publisher<T> * internalPublisher) {
        auto search = _subscriberMap.find(topicName);
        if (search == _subscriberMap.end()) {
            std::shared_ptr<DDSListener<T, M>> ddsListener = std::shared_ptr<DDSListener<T, M>>(new DDSListener<T, M>(useTake, internalPublisher, topicName));
            ddsReader->listener(ddsListener.get(), dds::core::status::StatusMask::data_available());
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(ddsListener);
            _subscriberMap[topicName] = internalPointer;
        }
    }

    /**
     * @brief registerToTopic
     * @param topicName
     * @param ddsReader
     * @param useTake
     * @param internalPublisher
     */
    template<class M>
    void unregisterFromTopic(std::string topicName,
                             dds::sub::DataReader<M> * ddsReader) {
        auto search = _subscriberMap.find(topicName);
        if (search == _subscriberMap.end()) {
            return;
        }
        ddsReader->listener(nullptr, dds::core::status::StatusMask::data_available());
        _subscriberMap.erase(search);

    }

private:
    std::map<std::string, std::shared_ptr<void>> _subscriberMap;
};
}
}
#endif
