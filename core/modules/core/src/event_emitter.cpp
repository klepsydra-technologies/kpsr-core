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

#include <klepsydra/core/event_emitter.h>
#include <stdexcept>

kpsr::EventEmitter::EventEmitter() {}

kpsr::EventEmitter::~EventEmitter() {}

unsigned int kpsr::EventEmitter::add_listener(const std::string & event_id, const std::string & listener_name, bool isOnce, std::function<void ()> cb)
{
    if (!cb)
    {
        throw std::invalid_argument("kpsr::EventEmitter::add_listener: No callbak provided.");
    }
    
    std::lock_guard<std::mutex> lock(mutex);

    unsigned int listener_id = ++last_listener;
    listeners.insert(std::make_pair(event_id, std::make_shared<Listener<>>(listener_id, isOnce, cb)));
    if (!isOnce) {
        _listenerStats.insert(std::make_pair(listener_id, std::make_shared<kpsr::SubscriptionStats>(listener_name, event_id, "EVENT_EMITTER")));
    }

    return listener_id;       
}

unsigned int kpsr::EventEmitter::on(const std::string & event_id, const std::string & listener_name, std::function<void ()> cb)
{
    return add_listener(event_id, listener_name, false, cb);
}

unsigned int kpsr::EventEmitter::once(const std::string & event_id, std::function<void ()> cb)
{
    return add_listener(event_id, "once", true, cb);
}

void kpsr::EventEmitter::remove_listener(unsigned int listener_id)
{
    std::lock_guard<std::mutex> lock(mutex);

    auto i = std::find_if(listeners.begin(), listeners.end(), [&] (std::pair<const std::string, std::shared_ptr<ListenerBase>> p) {
        return p.second->id == listener_id;
    });
    if (i != listeners.end())
    {
        listeners.erase(i);
        _listenerStats.erase(listener_id);
    }
    else
    {
        throw std::invalid_argument("kpsr::EventEmitter::remove_listener: Invalid listener id.");
    }
}
