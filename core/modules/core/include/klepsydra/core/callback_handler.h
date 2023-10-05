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

#ifndef CALLBACK_HANDLER_H
#define CALLBACK_HANDLER_H

#include <functional>
#include <mutex>
#include <tuple>
#include <utility>
#include <vector>

#include <spdlog/spdlog.h>

#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/subscriber.h>

namespace kpsr {

template<class Request, class Reply>
/**
 * @brief The CallbackHandler class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    CallbackHandler(const std::string &name,
                    Publisher<Request> *publisher,
                    Subscriber<Reply> *subscriber,
                    std::function<bool(const Request &, const Reply &)> correlationFunction)
        : _publisher(publisher)
        , _subscriber(subscriber)
        , _correlationFunction(correlationFunction)
        , _name(name)
    {
        std::function<void(const Reply &)> replyListener =
            std::bind(&CallbackHandler::onReplyReceived, this, std::placeholders::_1);
        _subscriber->registerListener(_name, replyListener);
    }

    virtual ~CallbackHandler() { _subscriber->removeListener(_name); }

    /**
     * @brief requestAndReply
     * @param request Event to publish
     * @param callback std::function to invoke when the reply is received.
     */
    virtual void requestAndReply(const Request &request,
                                 const std::function<void(const Reply &)> &callback)
    {
        std::pair<Request, std::function<void(const Reply &)>> pair = std::make_pair(request,
                                                                                     callback);
        _requestCallbackVector.push_back(pair);
        _publisher->publish(request);
    }

protected:
    virtual void onReplyReceived(const Reply &reply)
    {
        for (auto it = _requestCallbackVector.begin(); it != _requestCallbackVector.end();) {
            if (_correlationFunction((*it).first, reply)) {
                (*it).second(reply);
                _requestCallbackVector.erase(it);
                return;
            }
            ++it;
        }
        spdlog::info(
            "CallbackHandler::onReplyReceived. Reply not mapped. Callback name: {}. _publisher: {}",
            _name,
            _publisher->name);
    }

private:
    Publisher<Request> *_publisher;
    Subscriber<Reply> *_subscriber;
    bool _threadSafe;
    std::function<bool(const Request &, const Reply &)> _correlationFunction;
    std::string _name;
    std::vector<std::pair<Request, std::function<void(const Reply &)>>> _requestCallbackVector;
    mutable std::mutex _mutex;
};

template<class Request, class Reply>
/**
 * @brief The MultiThreadCallbackHandler class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    MultiThreadCallbackHandler(
        const std::string &name,
        Publisher<Request> *publisher,
        Subscriber<Reply> *subscriber,
        std::function<bool(const Request &, const Reply &)> correlationFunction)
        : CallbackHandler<Request, Reply>(name, publisher, subscriber, correlationFunction)
    {}

    virtual ~MultiThreadCallbackHandler() {}

    /**
     * @brief requestAndReply
     * @param request Event to publish
     * @param callback std::function to invoke when the reply is received.
     */
    void requestAndReply(const Request &request,
                         const std::function<void(const Reply &)> &callback) override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        CallbackHandler<Request, Reply>::requestAndReply(request, callback);
    }

protected:
    void onReplyReceived(const Reply &reply) override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        CallbackHandler<Request, Reply>::onReplyReceived(reply);
    }

private:
    mutable std::mutex _mutex;
};
} // namespace kpsr
#endif // CALLBACK_HANDLER_H
