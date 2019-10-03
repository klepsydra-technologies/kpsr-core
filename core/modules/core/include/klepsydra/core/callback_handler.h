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

#ifndef CALLBACK_HANDLER_H
#define CALLBACK_HANDLER_H

#include <functional>
#include <tuple>
#include <vector>
#include <iostream>
#include <utility>
#include <mutex>

#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>

namespace kpsr
{

template <class Request, class Reply>
/**
 * @brief The CallbackHandler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class is a helper class that facilitates the callback pattern with the Klepszdra API.
 * The main API publishes a message to a publisher and listens to responses in another queue and perform the callback action
 * when the correlation function returns true. The following is an example of how to use this facility class.
@code
class TestRequest {
public:
    int id;
    std::string message;
};

class TestReply {
public:
    int id;
    bool ack;
};

class Application {

   void callbackRun() {
    kpsr::CallbackHandler<TestRequest, TestReply> callbackHandler(
      "callback_example",
      _requestPublisher,
      _replySubcriber,
      [] const TestRequest & request, const TestReply & reply){ return request.id == reply.id; });

    kpsr::mem::CacheListener<TestReply> replyListener;

    TestRequest request;
    request.id = 1;
    request.message = "hola";

    callbackHandler.requestAndReply(request, replyListenerFunction.cacheListenerFunction);
   }

private:
    kpsr::Publisher<TestRequest> * _requestPublisher;
    kpsr::Subscriber<TestReply> * _replySubcriber;

@endcode
 *
 */
class CallbackHandler
{
public:
    /**
     * @brief CallbackHandler constructor.
     *
     * @param name It is used to register the reply listener in the subscriber.
     * @param publisher Used to publish request to.
     * @param subscriber Used to register the reply listener on.
     * @param correlationFunction Used to determine if a request and a reply are correlated.
     */
    CallbackHandler(std::string name,
                    Publisher<Request> * publisher,
                    Subscriber<Reply> * subscriber,
                    std::function<bool(const Request &, const Reply &)> correlationFunction)
        : _publisher(publisher)
        , _subscriber(subscriber)
        , _correlationFunction(correlationFunction)
        , _name(name) {
        std::function<void(const Reply &)> replyListener = std::bind(&CallbackHandler::onReplyReceived, this, std::placeholders::_1);
        _subscriber->registerListener(_name, replyListener);
    }

    virtual ~CallbackHandler() {
        _subscriber->removeListener(_name);
    }

    /**
     * @brief requestAndReply
     * @param request Event to publish
     * @param callback std::function to invoke when the reply is received.
     */
    virtual void requestAndReply(const Request & request, const std::function<void(const Reply &)> & callback) {
        std::pair<Request, std::function<void(const Reply &)>> pair = std::make_pair(request, callback);
        _requestCallbackVector.push_back(pair);
        _publisher->publish(request);
    }

protected:

    virtual void onReplyReceived(const Reply & reply) {
        for(auto it = _requestCallbackVector.begin(); it != _requestCallbackVector.end();) {
            if (_correlationFunction((*it).first, reply)) {
                (*it).second(reply);
                _requestCallbackVector.erase(it);
                return;
            }
            ++it;
        }
        std::cout << "CallbackHandler::onReplyReceived. Reply not mapped. Callback name: " << _name
                  << ". _publisher: " << _publisher->_publicationStats._name << std::endl;
    }

private:
    Publisher<Request> * _publisher;
    Subscriber<Reply> * _subscriber;
    bool _threadSafe;
    std::function<bool(const Request &, const Reply &)> _correlationFunction;
    std::string _name;
    std::vector<std::pair<Request, std::function<void(const Reply &)>>> _requestCallbackVector;
    mutable std::mutex _mutex;

};


template <class Request, class Reply>
/**
 * @brief The MultiThreadCallbackHandler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class is a helper class that facilitates the callback pattern with the Klepszdra API.
 * The main API publishes a message to a publisher and listens to responses in another queue and perform the callback action
 * when the correlation function returns true. The following is an example of how to use this facility class.
@code
class TestRequest {
public:
    int id;
    std::string message;
};

class TestReply {
public:
    int id;
    bool ack;
};

class Application {

   void callbackRun() {
    kpsr::MultiThreadCallbackHandler<TestRequest, TestReply> callbackHandler(
      "callback_example",
      _requestPublisher,
      _replySubcriber,
      [] const TestRequest & request, const TestReply & reply){ return request.id == reply.id; });

    kpsr::mem::CacheListener<TestReply> replyListener;

    TestRequest request;
    request.id = 1;
    request.message = "hola";

    callbackHandler.requestAndReply(request, replyListenerFunction.cacheListenerFunction);
   }

private:
    kpsr::Publisher<TestRequest> * _requestPublisher;
    kpsr::Subscriber<TestReply> * _replySubcriber;

@endcode
 *
 */
class MultiThreadCallbackHandler : public CallbackHandler<Request, Reply>
{
public:
    /**
     * @brief CallbackHandler constructor.
     *
     * @param name It is used to register the reply listener in the subscriber.
     * @param publisher Used to publish request to.
     * @param subscriber Used to register the reply listener on.
     * @param correlationFunction Used to determine if a request and a reply are correlated.
     */
    MultiThreadCallbackHandler(std::string name,
                               Publisher<Request> * publisher,
                               Subscriber<Reply> * subscriber,
                               std::function<bool(const Request &, const Reply &)> correlationFunction)
        : CallbackHandler<Request, Reply>(name, publisher, subscriber, correlationFunction)
    {}

    virtual ~MultiThreadCallbackHandler() {}

    /**
     * @brief requestAndReply
     * @param request Event to publish
     * @param callback std::function to invoke when the reply is received.
     */
    void requestAndReply(const Request & request, const std::function<void(const Reply &)> & callback) override {
        std::lock_guard<std::mutex> lock (_mutex);
        CallbackHandler<Request, Reply>::requestAndReply(request, callback);
    }

protected:

    void onReplyReceived(const Reply & reply) override {
        std::lock_guard<std::mutex> lock (_mutex);
        CallbackHandler<Request, Reply>::onReplyReceived(reply);
    }

private:
    mutable std::mutex _mutex;

};
}
#endif // CALLBACK_HANDLER_H
