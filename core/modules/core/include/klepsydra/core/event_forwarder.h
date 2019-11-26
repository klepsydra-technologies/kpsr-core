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

#ifndef EVENT_FORWARDER_H
#define EVENT_FORWARDER_H

#include <klepsydra/core/publisher.h>

namespace kpsr
{
template <class T>

/*!
 * @brief The EventForwarder class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class provide a facility method to forward an event received on a subscriber. This is mainly used when data needs to be
 * used in a memory middleware and also published to some middleware (e.g., images, state machine publicstates, etc.)
*/
class EventForwarder
{
public:
    /*!
     * @brief EventForwarder
     * @param publisher
     */
    EventForwarder(Publisher<T> * publisher) {
        _publisher = publisher;
    }

    /*!
     * @brief onMessageReceived
     * @param event
     */
    void onMessageReceived(const T& event) {
        _publisher->publish(event);
    }

private:
    Publisher<T> * _publisher;
};
}
#endif
