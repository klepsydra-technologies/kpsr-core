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

#ifndef CALLBACK_HANDLER_H
#define CALLBACK_HANDLER_H

#include <functional>
#include <tuple>
#include <vector>
#include <utility>
#include <mutex>

#include <spdlog/spdlog.h>


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
    CallbackHandler(const std::string & name,
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
        spdlog::info("CallbackHandler::onReplyReceived. Reply not mapped. Callback name: {}. _publisher: {}", _name, _publisher->_publicationStats._name);
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
    MultiThreadCallbackHandler(const std::string & name,
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
