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

#include <klepsydra/core/event_emitter.h>
#include <stdexcept>

kpsr::EventEmitter::EventEmitter() {}

kpsr::EventEmitter::~EventEmitter() {}

unsigned int kpsr::EventEmitter::add_listener(std::string event_id, std::string listener_name, bool isOnce, std::function<void ()> cb)
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

unsigned int kpsr::EventEmitter::on(std::string event_id, std::string listener_name, std::function<void ()> cb)
{
    return add_listener(event_id, listener_name, false, cb);
}

unsigned int kpsr::EventEmitter::once(std::string event_id, std::function<void ()> cb)
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
