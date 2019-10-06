<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-application` 

This group of classes contains the API for the application development. That means that the client codes that involves the actual logic of the application, should use this and only this API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`enum `[`SystemEventData`](#group__kpsr-application_1ga59297e4470b81aa9e283471cc0a4bdd1)            | The SystemEventData struct.
`class `[`kpsr::CallbackHandler`](#classkpsr_1_1CallbackHandler) | The [CallbackHandler](#classkpsr_1_1CallbackHandler) class.
`class `[`kpsr::MultiThreadCallbackHandler`](#classkpsr_1_1MultiThreadCallbackHandler) | The [MultiThreadCallbackHandler](#classkpsr_1_1MultiThreadCallbackHandler) class.
`class `[`kpsr::Environment`](#classkpsr_1_1Environment) | The [Environment](#classkpsr_1_1Environment) interface.
`class `[`kpsr::EventForwarder`](#classkpsr_1_1EventForwarder) | The [EventForwarder](#classkpsr_1_1EventForwarder) class.
`class `[`kpsr::EventTransformForwarder`](#classkpsr_1_1EventTransformForwarder) | The [EventTransformForwarder](#classkpsr_1_1EventTransformForwarder) class.
`class `[`kpsr::ManagedService`](#classkpsr_1_1ManagedService) | The [ManagedService](#classkpsr_1_1ManagedService) class.
`class `[`kpsr::Publisher`](#classkpsr_1_1Publisher) | The [Publisher](#classkpsr_1_1Publisher) class.
`class `[`kpsr::Service`](#classkpsr_1_1Service) | The [Service](#classkpsr_1_1Service) class.
`class `[`kpsr::SmartObjectPool`](#classkpsr_1_1SmartObjectPool) | The [SmartObjectPool](#classkpsr_1_1SmartObjectPool) class.
`class `[`kpsr::Subscriber`](#classkpsr_1_1Subscriber) | The [Subscriber](#classkpsr_1_1Subscriber) class.
`struct `[`kpsr::Sensor`](#structkpsr_1_1Sensor) | The [Sensor](#structkpsr_1_1Sensor) struct.
`struct `[`kpsr::SmartObjectPool::ReturnToPool_Deleter`](#structkpsr_1_1SmartObjectPool_1_1ReturnToPool__Deleter) | 

## Members

#### `enum `[`SystemEventData`](#group__kpsr-application_1ga59297e4470b81aa9e283471cc0a4bdd1) 

 Values                         | Descriptions                                
--------------------------------|---------------------------------------------
Start            | 
Stop            | 
Idle            | 

The SystemEventData struct.

Klepsydra Technologies 2019-2020.

2.1.0

System Event Data enum. These are events that drive the running status of a slave service.

# class `kpsr::CallbackHandler` 

The [CallbackHandler](#classkpsr_1_1CallbackHandler) class.

Klepsydra Technologies 2019-2020.

2.1.0

This class is a helper class that facilitates the callback pattern with the Klepszdra API. The main API publishes a message to a publisher and listens to responses in another queue and perform the callback action when the correlation function returns true. The following is an example of how to use this facility class. 
```cpp
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
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`CallbackHandler`](#classkpsr_1_1CallbackHandler_1aedf75765099878fa112708a3c5f93141)`(std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< Request > * publisher,`[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< Reply > * subscriber,std::function< bool(const Request &, const Reply &)> correlationFunction)` | [CallbackHandler](#classkpsr_1_1CallbackHandler) constructor.
`public inline virtual  `[`~CallbackHandler`](#classkpsr_1_1CallbackHandler_1aa2122007d769d4d51a1dc8183c819981)`()` | 
`public inline virtual void `[`requestAndReply`](#classkpsr_1_1CallbackHandler_1a5683abcf8ea233724e98ab54b4b8bbe3)`(const Request & request,const std::function< void(const Reply &)> & callback)` | requestAndReply
`protected inline virtual void `[`onReplyReceived`](#classkpsr_1_1CallbackHandler_1af7ac71dd83897fe6686756c56ca030de)`(const Reply & reply)` | 

## Members

#### `public inline  `[`CallbackHandler`](#classkpsr_1_1CallbackHandler_1aedf75765099878fa112708a3c5f93141)`(std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< Request > * publisher,`[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< Reply > * subscriber,std::function< bool(const Request &, const Reply &)> correlationFunction)` 

[CallbackHandler](#classkpsr_1_1CallbackHandler) constructor.

#### Parameters
* `name` It is used to register the reply listener in the subscriber. 

* `publisher` Used to publish request to. 

* `subscriber` Used to register the reply listener on. 

* `correlationFunction` Used to determine if a request and a reply are correlated.

#### `public inline virtual  `[`~CallbackHandler`](#classkpsr_1_1CallbackHandler_1aa2122007d769d4d51a1dc8183c819981)`()` 

#### `public inline virtual void `[`requestAndReply`](#classkpsr_1_1CallbackHandler_1a5683abcf8ea233724e98ab54b4b8bbe3)`(const Request & request,const std::function< void(const Reply &)> & callback)` 

requestAndReply

#### Parameters
* `request` Event to publish 

* `callback` std::function to invoke when the reply is received.

#### `protected inline virtual void `[`onReplyReceived`](#classkpsr_1_1CallbackHandler_1af7ac71dd83897fe6686756c56ca030de)`(const Reply & reply)` 

# class `kpsr::MultiThreadCallbackHandler` 

```
class kpsr::MultiThreadCallbackHandler
  : public kpsr::CallbackHandler< Request, Reply >
```  

The [MultiThreadCallbackHandler](#classkpsr_1_1MultiThreadCallbackHandler) class.

Klepsydra Technologies 2019-2020.

2.1.0

This class is a helper class that facilitates the callback pattern with the Klepszdra API. The main API publishes a message to a publisher and listens to responses in another queue and perform the callback action when the correlation function returns true. The following is an example of how to use this facility class. 
```cpp
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
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`MultiThreadCallbackHandler`](#classkpsr_1_1MultiThreadCallbackHandler_1aaf0308141a26ea0d4dc29a0dbcee40f8)`(std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< Request > * publisher,`[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< Reply > * subscriber,std::function< bool(const Request &, const Reply &)> correlationFunction)` | [CallbackHandler](api-kpsr-application.md#classkpsr_1_1CallbackHandler) constructor.
`public inline virtual  `[`~MultiThreadCallbackHandler`](#classkpsr_1_1MultiThreadCallbackHandler_1a5fa99496614e0e2f5eeb53cb0ec5a52d)`()` | 
`public inline virtual void `[`requestAndReply`](#classkpsr_1_1MultiThreadCallbackHandler_1ab5545065b5887b3d3d5d4fc8c62db36c)`(const Request & request,const std::function< void(const Reply &)> & callback)` | requestAndReply
`protected inline virtual void `[`onReplyReceived`](#classkpsr_1_1MultiThreadCallbackHandler_1a7c044a691870e61ec41c9c818f4a88db)`(const Reply & reply)` | 

## Members

#### `public inline  `[`MultiThreadCallbackHandler`](#classkpsr_1_1MultiThreadCallbackHandler_1aaf0308141a26ea0d4dc29a0dbcee40f8)`(std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< Request > * publisher,`[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< Reply > * subscriber,std::function< bool(const Request &, const Reply &)> correlationFunction)` 

[CallbackHandler](api-kpsr-application.md#classkpsr_1_1CallbackHandler) constructor.

#### Parameters
* `name` It is used to register the reply listener in the subscriber. 

* `publisher` Used to publish request to. 

* `subscriber` Used to register the reply listener on. 

* `correlationFunction` Used to determine if a request and a reply are correlated.

#### `public inline virtual  `[`~MultiThreadCallbackHandler`](#classkpsr_1_1MultiThreadCallbackHandler_1a5fa99496614e0e2f5eeb53cb0ec5a52d)`()` 

#### `public inline virtual void `[`requestAndReply`](#classkpsr_1_1MultiThreadCallbackHandler_1ab5545065b5887b3d3d5d4fc8c62db36c)`(const Request & request,const std::function< void(const Reply &)> & callback)` 

requestAndReply

#### Parameters
* `request` Event to publish 

* `callback` std::function to invoke when the reply is received.

#### `protected inline virtual void `[`onReplyReceived`](#classkpsr_1_1MultiThreadCallbackHandler_1a7c044a691870e61ec41c9c818f4a88db)`(const Reply & reply)` 

# class `kpsr::Environment` 

The [Environment](#classkpsr_1_1Environment) interface.

Klepsydra Technologies 2019-2020.

2.1.0

This class is a facility class that was born to isolate the ROS environment from the application classes so that they can stay agnostic of ROS. There are several implementations including a mock one based on memory maps that can be used for unit testing.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public void `[`getPropertyString`](#classkpsr_1_1Environment_1a486a13feca456cfa94262fe1c4579b06)`(const std::string key,std::string & value)` | getPropertyString
`public void `[`getPropertyInt`](#classkpsr_1_1Environment_1acffdc7d0f1b8e2c807eb3818ddf4d899)`(const std::string key,int & value)` | getPropertyInt
`public void `[`getPropertyFloat`](#classkpsr_1_1Environment_1aefd29e485bbdb2d70013e5c5d2a3af23)`(const std::string key,float & value)` | getPropertyFloat
`public void `[`getPropertyBool`](#classkpsr_1_1Environment_1ab37c893ec1d29dcf16e8739bb941ac4d)`(const std::string key,bool & value)` | getPropertyBool
`public void `[`setPropertyString`](#classkpsr_1_1Environment_1a4a7bbebaa7776844ef25716c4ba1b95e)`(const std::string key,const std::string value)` | setPropertyString
`public void `[`setPropertyInt`](#classkpsr_1_1Environment_1a31d6199ef673c997d44fc8a0fb69dc53)`(const std::string key,const int & value)` | setPropertyInt
`public void `[`setPropertyFloat`](#classkpsr_1_1Environment_1a7a5d80de6bd09744bbb48c0669aad269)`(const std::string key,const float & value)` | setPropertyFloat
`public void `[`setPropertyBool`](#classkpsr_1_1Environment_1af989ce172d572fc745dd7dea48d9a71a)`(const std::string key,const bool & value)` | setPropertyBool
`public void `[`persist`](#classkpsr_1_1Environment_1a0e5d3a16559b140c32d779410d325500)`()` | persist

## Members

#### `public void `[`getPropertyString`](#classkpsr_1_1Environment_1a486a13feca456cfa94262fe1c4579b06)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public void `[`getPropertyInt`](#classkpsr_1_1Environment_1acffdc7d0f1b8e2c807eb3818ddf4d899)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public void `[`getPropertyFloat`](#classkpsr_1_1Environment_1aefd29e485bbdb2d70013e5c5d2a3af23)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public void `[`getPropertyBool`](#classkpsr_1_1Environment_1ab37c893ec1d29dcf16e8739bb941ac4d)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public void `[`setPropertyString`](#classkpsr_1_1Environment_1a4a7bbebaa7776844ef25716c4ba1b95e)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public void `[`setPropertyInt`](#classkpsr_1_1Environment_1a31d6199ef673c997d44fc8a0fb69dc53)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public void `[`setPropertyFloat`](#classkpsr_1_1Environment_1a7a5d80de6bd09744bbb48c0669aad269)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public void `[`setPropertyBool`](#classkpsr_1_1Environment_1af989ce172d572fc745dd7dea48d9a71a)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public void `[`persist`](#classkpsr_1_1Environment_1a0e5d3a16559b140c32d779410d325500)`()` 

persist

This method is used to invoke persistence of the properties. I might be used in cases where the persistance does not happen automatically, for example in the kpsr::YamlEnvironment.

# class `kpsr::EventForwarder` 

The [EventForwarder](#classkpsr_1_1EventForwarder) class.

Klepsydra Technologies 2019-2020.

2.1.0

This class provide a facility method to forward an event received on a subscriber. This is mainly used when data needs to be used in a memory middleware and also published to some middleware (e.g., images, state machine publicstates, etc.)

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventForwarder`](#classkpsr_1_1EventForwarder_1aa057148350620965ee6c1afd33574f69)`(`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * publisher)` | [EventForwarder](#classkpsr_1_1EventForwarder).
`public inline void `[`onMessageReceived`](#classkpsr_1_1EventForwarder_1a3e5135d3dcc31c8c8ccb94bccbda01a7)`(const T & event)` | onMessageReceived

## Members

#### `public inline  `[`EventForwarder`](#classkpsr_1_1EventForwarder_1aa057148350620965ee6c1afd33574f69)`(`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * publisher)` 

[EventForwarder](#classkpsr_1_1EventForwarder).

#### Parameters
* `publisher`

#### `public inline void `[`onMessageReceived`](#classkpsr_1_1EventForwarder_1a3e5135d3dcc31c8c8ccb94bccbda01a7)`(const T & event)` 

onMessageReceived

#### Parameters
* `event`

# class `kpsr::EventTransformForwarder` 

The [EventTransformForwarder](#classkpsr_1_1EventTransformForwarder) class.

Klepsydra Technologies 2019-2020.

2.1.0

An optimised forwarding helper class. Its purpose is to optimise memory allocation and callstack when processing events, transforming and forwarding them.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::function< void(const T &)> `[`forwarderListenerFunction`](#classkpsr_1_1EventTransformForwarder_1a51a4da27d03e2fe2949dbc4b83c0b037) | forwarderListenerFunction std::function to be added to the source subcriber.
`public inline  `[`EventTransformForwarder`](#classkpsr_1_1EventTransformForwarder_1abc289af227286530635b0ddbeca430bf)`(std::function< void(const T &, U &)> transformFunction,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< U > * destPublisher)` | [EventTransformForwarder](#classkpsr_1_1EventTransformForwarder).
`public inline void `[`onEventReceived`](#classkpsr_1_1EventTransformForwarder_1a43ee5a14a6a55063aff604facd12208f)`(const T & event)` | onEventReceived

## Members

#### `public std::function< void(const T &)> `[`forwarderListenerFunction`](#classkpsr_1_1EventTransformForwarder_1a51a4da27d03e2fe2949dbc4b83c0b037) 

forwarderListenerFunction std::function to be added to the source subcriber.

#### `public inline  `[`EventTransformForwarder`](#classkpsr_1_1EventTransformForwarder_1abc289af227286530635b0ddbeca430bf)`(std::function< void(const T &, U &)> transformFunction,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< U > * destPublisher)` 

[EventTransformForwarder](#classkpsr_1_1EventTransformForwarder).

#### Parameters
* `transformFunction` function that process and event and transforms it into another object. 

* `destPublisher` forwarding publisher.

#### `public inline void `[`onEventReceived`](#classkpsr_1_1EventTransformForwarder_1a43ee5a14a6a55063aff604facd12208f)`(const T & event)` 

onEventReceived

#### Parameters
* `event`

# class `kpsr::ManagedService` 

```
class kpsr::ManagedService
  : public kpsr::Service
```  

The [ManagedService](#classkpsr_1_1ManagedService) class.

Klepsydra Technologies 2019-2020.

2.1.0

There are two types of services: master and slave. They are distinguised in construction time by passing the parameter ''systemStatusEventSubscriber'' as null or not null respectively.

Slave services in the other hand only start running when commanded by a system service. A system service is nothing but a master service that can publish events of the System Event Data type.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ManagedService`](#classkpsr_1_1ManagedService_1a8b118bb43b7675d026e483becf90e1d4)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * environment,`[`Subscriber](api-kpsr-application.md#classkpsr_1_1Subscriber)< [SystemEventData`](api-undefined.md#group__kpsr-application_1ga59297e4470b81aa9e283471cc0a4bdd1)` > * systemStatusEventSubscriber,std::string serviceName)` | [ManagedService](#classkpsr_1_1ManagedService).

## Members

#### `public inline  `[`ManagedService`](#classkpsr_1_1ManagedService_1a8b118bb43b7675d026e483becf90e1d4)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * environment,`[`Subscriber](api-kpsr-application.md#classkpsr_1_1Subscriber)< [SystemEventData`](api-undefined.md#group__kpsr-application_1ga59297e4470b81aa9e283471cc0a4bdd1)` > * systemStatusEventSubscriber,std::string serviceName)` 

[ManagedService](#classkpsr_1_1ManagedService).

#### Parameters
* `environment` 

* `systemStatusEventSubscriber` start and stop will be handled by the master service. 

* `serviceName`

# class `kpsr::Publisher` 

The [Publisher](#classkpsr_1_1Publisher) class.

Klepsydra Technologies 2019-2020.

2.1.0

Main publisher API. Applications using Klepsydra should use pointers to this class in order to publish event to either other classes (intra process) or to the middleware (inter process)

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public `[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` `[`_publicationStats`](#classkpsr_1_1Publisher_1a137a479da4fee5afa4160335988b2e63) | _publicationStats
`public inline  `[`Publisher`](#classkpsr_1_1Publisher_1acbcf52536c6bd6812040a235d0ff700a)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,const std::string type)` | [Publisher](#classkpsr_1_1Publisher).
`public inline void `[`publish`](#classkpsr_1_1Publisher_1a4d7fd841ee5ac6ebef9fef4ab91b91f7)`(const T & event)` | publish
`public inline void `[`publish`](#classkpsr_1_1Publisher_1ab8c192b8582e19d61b142f55710ae487)`(std::shared_ptr< const T > event)` | publish without copy
`public void `[`processAndPublish`](#classkpsr_1_1Publisher_1a18046c58c39cc1a4f659d2053356f5dd)`(std::function< void(T &)> process)` | processAndPublish

## Members

#### `public `[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` `[`_publicationStats`](#classkpsr_1_1Publisher_1a137a479da4fee5afa4160335988b2e63) 

_publicationStats

#### `public inline  `[`Publisher`](#classkpsr_1_1Publisher_1acbcf52536c6bd6812040a235d0ff700a)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,const std::string type)` 

[Publisher](#classkpsr_1_1Publisher).

#### Parameters
* `container` 

* `name` 

* `type`

#### `public inline void `[`publish`](#classkpsr_1_1Publisher_1a4d7fd841ee5ac6ebef9fef4ab91b91f7)`(const T & event)` 

publish

#### Parameters
* `event`

#### `public inline void `[`publish`](#classkpsr_1_1Publisher_1ab8c192b8582e19d61b142f55710ae487)`(std::shared_ptr< const T > event)` 

publish without copy

#### Parameters
* `event`

#### `public void `[`processAndPublish`](#classkpsr_1_1Publisher_1a18046c58c39cc1a4f659d2053356f5dd)`(std::function< void(T &)> process)` 

processAndPublish

#### Parameters
* `process`

# class `kpsr::Service` 

The [Service](#classkpsr_1_1Service) class.

Klepsydra Technologies 2019-2020.

2.1.0

[Service](#classkpsr_1_1Service) Interface. Custumer applications using Klepsydra that need remote administration, access to configuration perform regular task, etc should implement this interface.

The only pure virtual methods to implement are start, stop and execute. Master services start running as soon as the application is _started and should not implement the start method. There are some stat gathering helper functions that are used within the container interface.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public `[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * `[`_environment`](#classkpsr_1_1Service_1afe999c1d5a3183b858a89cbd46951443) | _environment
`public `[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` `[`_serviceStats`](#classkpsr_1_1Service_1a1c2ec593ec97e91852ba2bc96d7ae439) | _serviceStats
`public bool `[`_isMaster`](#classkpsr_1_1Service_1a818a4bbe74ee38757f17f0b20e3db00d) | _isMaster
`public std::atomic< bool > `[`_started`](#classkpsr_1_1Service_1a6cef64c570b4b198087a7b6282f61d3c) | _started
`public inline  `[`Service`](#classkpsr_1_1Service_1aa8d67e40fd3209db637feae669dc77f9)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * environment,std::string serviceName,bool isMaster)` | [Service](#classkpsr_1_1Service).
`public inline virtual void `[`runOnce`](#classkpsr_1_1Service_1ab220963d214d1e1edc095465504b2673)`()` | runOnce
`public inline virtual void `[`startup`](#classkpsr_1_1Service_1a5b7611ce05533504a890e23d485ca347)`()` | startup
`public inline virtual void `[`shutdown`](#classkpsr_1_1Service_1a0bef30aa0f67e4a5f6f0ee5c8e18a106)`()` | shutdown
`public inline virtual std::map< std::string, std::string > `[`getMetadata`](#classkpsr_1_1Service_1aaa68b51176d3405d7b6330b37f8b840b)`()` | getMetadata
`protected void `[`execute`](#classkpsr_1_1Service_1a44505d9611f3e5e6c31d78d77c998385)`()` | 
`protected void `[`start`](#classkpsr_1_1Service_1aa0238be012876aeb96e0722c8b80eae7)`()` | 
`protected void `[`stop`](#classkpsr_1_1Service_1a8dfcdda7d795ff9deab6d671a6907e96)`()` | 

## Members

#### `public `[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * `[`_environment`](#classkpsr_1_1Service_1afe999c1d5a3183b858a89cbd46951443) 

_environment

#### `public `[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` `[`_serviceStats`](#classkpsr_1_1Service_1a1c2ec593ec97e91852ba2bc96d7ae439) 

_serviceStats

#### `public bool `[`_isMaster`](#classkpsr_1_1Service_1a818a4bbe74ee38757f17f0b20e3db00d) 

_isMaster

#### `public std::atomic< bool > `[`_started`](#classkpsr_1_1Service_1a6cef64c570b4b198087a7b6282f61d3c) 

_started

#### `public inline  `[`Service`](#classkpsr_1_1Service_1aa8d67e40fd3209db637feae669dc77f9)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * environment,std::string serviceName,bool isMaster)` 

[Service](#classkpsr_1_1Service).

#### Parameters
* `environment` 

* `serviceName` 

* `isMaster`

#### `public inline virtual void `[`runOnce`](#classkpsr_1_1Service_1ab220963d214d1e1edc095465504b2673)`()` 

runOnce

This public method is used to invoke the custom service execute method. Usually this method is invoked within a main application. E.g., in the Ros world, this will be invoked within the runOnce of Ros.

#### `public inline virtual void `[`startup`](#classkpsr_1_1Service_1a5b7611ce05533504a890e23d485ca347)`()` 

startup

This public method is used to invoke the custom service start method. Usually this method is invoked within a main application.

#### `public inline virtual void `[`shutdown`](#classkpsr_1_1Service_1a0bef30aa0f67e4a5f6f0ee5c8e18a106)`()` 

shutdown

This public method is used to invoke the custom service stop method. Usually this method is invoked within a main application.

#### `public inline virtual std::map< std::string, std::string > `[`getMetadata`](#classkpsr_1_1Service_1aaa68b51176d3405d7b6330b37f8b840b)`()` 

getMetadata

This method is intented to be used from the container. It provides metadata related to the service including status, running time, etc.

#### Returns

#### `protected void `[`execute`](#classkpsr_1_1Service_1a44505d9611f3e5e6c31d78d77c998385)`()` 

#### `protected void `[`start`](#classkpsr_1_1Service_1aa0238be012876aeb96e0722c8b80eae7)`()` 

#### `protected void `[`stop`](#classkpsr_1_1Service_1a8dfcdda7d795ff9deab6d671a6907e96)`()` 

# class `kpsr::SmartObjectPool` 

The [SmartObjectPool](#classkpsr_1_1SmartObjectPool) class.

Klepsydra Technologies 2019-2020.

2.1.0

Part of the memory allocation optimisation API, this class provide a generic way to create a pool objects. This class is available for developers to use, but it is also transparently integrated into the publishers for the different middleware, for the disruptor and in other places of the Klepsydra core toolset.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public long `[`objectPoolFails`](#classkpsr_1_1SmartObjectPool_1a296f6fce33b1e446eb68edd00f0c737d) | objectPoolFails
`public inline  `[`SmartObjectPool`](#classkpsr_1_1SmartObjectPool_1a0afd9d056e50fd621821500d14fcf803)`(int size,std::function< void(T &)> initializerFunction)` | [SmartObjectPool](#classkpsr_1_1SmartObjectPool).
`public inline virtual  `[`~SmartObjectPool`](#classkpsr_1_1SmartObjectPool_1ac174203f4395722d94f3c13453edaa94)`()` | 
`public inline ptr_type `[`acquire`](#classkpsr_1_1SmartObjectPool_1a5d97d9a2e66bbbfa62c62befe338fdf7)`()` | acquire fetch an element of the pool. It comes as a unique pointer. When no references are hanging, the object will return to the pool.
`public inline bool `[`empty`](#classkpsr_1_1SmartObjectPool_1a771fa5e881b754b87dc9bd28830c650f)`() const` | empty
`public inline size_t `[`size`](#classkpsr_1_1SmartObjectPool_1a0e89de3f55d1b2e8b455a4a6f8edf84d)`() const` | size
`typedef `[`ptr_type`](#classkpsr_1_1SmartObjectPool_1afba42b8698c87d85c3fd3627b4fa2c70) | 

## Members

#### `public long `[`objectPoolFails`](#classkpsr_1_1SmartObjectPool_1a296f6fce33b1e446eb68edd00f0c737d) 

objectPoolFails

#### `public inline  `[`SmartObjectPool`](#classkpsr_1_1SmartObjectPool_1a0afd9d056e50fd621821500d14fcf803)`(int size,std::function< void(T &)> initializerFunction)` 

[SmartObjectPool](#classkpsr_1_1SmartObjectPool).

#### Parameters
* `size` 

* `initializerFunction` optional std::function to initialise the objects.

#### `public inline virtual  `[`~SmartObjectPool`](#classkpsr_1_1SmartObjectPool_1ac174203f4395722d94f3c13453edaa94)`()` 

#### `public inline ptr_type `[`acquire`](#classkpsr_1_1SmartObjectPool_1a5d97d9a2e66bbbfa62c62befe338fdf7)`()` 

acquire fetch an element of the pool. It comes as a unique pointer. When no references are hanging, the object will return to the pool.

#### `public inline bool `[`empty`](#classkpsr_1_1SmartObjectPool_1a771fa5e881b754b87dc9bd28830c650f)`() const` 

empty

#### `public inline size_t `[`size`](#classkpsr_1_1SmartObjectPool_1a0e89de3f55d1b2e8b455a4a6f8edf84d)`() const` 

size

#### Returns

#### `typedef `[`ptr_type`](#classkpsr_1_1SmartObjectPool_1afba42b8698c87d85c3fd3627b4fa2c70) 

# class `kpsr::Subscriber` 

The [Subscriber](#classkpsr_1_1Subscriber) class.

Klepsydra Technologies 2019-2020.

2.1.0

[Subscriber](#classkpsr_1_1Subscriber) interface. It keeps track of the listeners that registers for events coming from a middleware.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public `[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * `[`_container`](#classkpsr_1_1Subscriber_1a8f9fcb88fa72205c190650ba80f5e3e0) | 
`public std::string `[`_name`](#classkpsr_1_1Subscriber_1a4c2eb6f6731e974a11ff740f6c0bac45) | 
`public std::string `[`_type`](#classkpsr_1_1Subscriber_1aff68bbe441ed3193f389d7341e89a026) | 
`public inline  `[`Subscriber`](#classkpsr_1_1Subscriber_1af05b20a9329991f084151b29c8c4f7fc)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::string type)` | [Subscriber](#classkpsr_1_1Subscriber).
`public void `[`registerListener`](#classkpsr_1_1Subscriber_1ad600a17849f6aa6498576c5624d3fdbd)`(const std::string name,const std::function< void(const T &)> listener)` | registerListener registers an std::function to be invoked everything an event is received.
`public void `[`registerListenerOnce`](#classkpsr_1_1Subscriber_1a1306ad33216abcf91ed8c67586d233d7)`(const std::function< void(const T &)> listener)` | registerListenerOnce registers an std::function to be invoked when an event is received. Once invoked this listeners is removed from the list of listeners.
`public void `[`removeListener`](#classkpsr_1_1Subscriber_1a77d557e37acd6cce40b1478a636ba08e)`(const std::string name)` | removeListener removes the listener from the list of active listeners.
`public std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1Subscriber_1a154ea6045f2aadb47d49772974f8b68c)`(const std::string name)` | getSubscriptionStats retrieves the performance information of the listener.

## Members

#### `public `[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * `[`_container`](#classkpsr_1_1Subscriber_1a8f9fcb88fa72205c190650ba80f5e3e0) 

#### `public std::string `[`_name`](#classkpsr_1_1Subscriber_1a4c2eb6f6731e974a11ff740f6c0bac45) 

#### `public std::string `[`_type`](#classkpsr_1_1Subscriber_1aff68bbe441ed3193f389d7341e89a026) 

#### `public inline  `[`Subscriber`](#classkpsr_1_1Subscriber_1af05b20a9329991f084151b29c8c4f7fc)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::string type)` 

[Subscriber](#classkpsr_1_1Subscriber).

#### Parameters
* `container` 

* `name` 

* `type`

#### `public void `[`registerListener`](#classkpsr_1_1Subscriber_1ad600a17849f6aa6498576c5624d3fdbd)`(const std::string name,const std::function< void(const T &)> listener)` 

registerListener registers an std::function to be invoked everything an event is received.

#### Parameters
* `name` with which the listener is registered. 

* `listener` function to be invoked for an event.

#### `public void `[`registerListenerOnce`](#classkpsr_1_1Subscriber_1a1306ad33216abcf91ed8c67586d233d7)`(const std::function< void(const T &)> listener)` 

registerListenerOnce registers an std::function to be invoked when an event is received. Once invoked this listeners is removed from the list of listeners.

#### Parameters
* `listener` function to be invoked for an event.

#### `public void `[`removeListener`](#classkpsr_1_1Subscriber_1a77d557e37acd6cce40b1478a636ba08e)`(const std::string name)` 

removeListener removes the listener from the list of active listeners.

#### Parameters
* `name`

#### `public std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1Subscriber_1a154ea6045f2aadb47d49772974f8b68c)`(const std::string name)` 

getSubscriptionStats retrieves the performance information of the listener.

#### Parameters
* `name`

# struct `kpsr::Sensor` 

The [Sensor](#structkpsr_1_1Sensor) struct.

Klepsydra Technologies 2019-2020.

2.1.0

This class is the base class for all sensor data available in Klepsydra. This includes: images, positioning, laser data, etc. It has a similar structure to the ROS [sensor_msgs Package](http://wiki.ros.org/sensor_msgs). However, at the same time it contains the same API as most Klepsydra POCO including a clone method, a constructor based on another instance and a constructor receiving all fields.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::string `[`frameId`](#structkpsr_1_1Sensor_1aceff76a12817db7b43eee35611575026) | frameId
`public int `[`seq`](#structkpsr_1_1Sensor_1a50b0c96f26651819ab904e7e98d21153) | seq
`public long `[`timestamp`](#structkpsr_1_1Sensor_1ae418bcb74e3cecc7f438d785d1e4f495) | timestamp
`public inline  `[`Sensor`](#structkpsr_1_1Sensor_1ab2e0991262dd756a41dabf3f12bc6949)`()` | [Sensor](#structkpsr_1_1Sensor).
`public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a6c86aa94074333b368d5f05e91e80a76)`(std::string frameId,int seq,long timestamp)` | [Sensor](#structkpsr_1_1Sensor).
`public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a9160499526c71a0b976d95289faacabb)`(std::string frameId,int seq)` | [Sensor](#structkpsr_1_1Sensor).
`public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a15822931564b532a3d5dbc1e08a86c49)`(const `[`Sensor`](#structkpsr_1_1Sensor)` & that)` | [Sensor](#structkpsr_1_1Sensor).
`public inline void `[`clone`](#structkpsr_1_1Sensor_1a9ea90411857a95c8d363867983d89fcd)`(const `[`Sensor`](#structkpsr_1_1Sensor)` & that)` | clone

## Members

#### `public std::string `[`frameId`](#structkpsr_1_1Sensor_1aceff76a12817db7b43eee35611575026) 

frameId

#### `public int `[`seq`](#structkpsr_1_1Sensor_1a50b0c96f26651819ab904e7e98d21153) 

seq

#### `public long `[`timestamp`](#structkpsr_1_1Sensor_1ae418bcb74e3cecc7f438d785d1e4f495) 

timestamp

#### `public inline  `[`Sensor`](#structkpsr_1_1Sensor_1ab2e0991262dd756a41dabf3f12bc6949)`()` 

[Sensor](#structkpsr_1_1Sensor).

#### `public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a6c86aa94074333b368d5f05e91e80a76)`(std::string frameId,int seq,long timestamp)` 

[Sensor](#structkpsr_1_1Sensor).

#### Parameters
* `frameId` 

* `seq` 

* `timestamp`

#### `public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a9160499526c71a0b976d95289faacabb)`(std::string frameId,int seq)` 

[Sensor](#structkpsr_1_1Sensor).

#### Parameters
* `frameId` 

* `seq`

#### `public inline  `[`Sensor`](#structkpsr_1_1Sensor_1a15822931564b532a3d5dbc1e08a86c49)`(const `[`Sensor`](#structkpsr_1_1Sensor)` & that)` 

[Sensor](#structkpsr_1_1Sensor).

#### Parameters
* `that`

#### `public inline void `[`clone`](#structkpsr_1_1Sensor_1a9ea90411857a95c8d363867983d89fcd)`(const `[`Sensor`](#structkpsr_1_1Sensor)` & that)` 

clone

#### Parameters
* `that`

# struct `kpsr::SmartObjectPool::ReturnToPool_Deleter` 

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  explicit `[`ReturnToPool_Deleter`](#structkpsr_1_1SmartObjectPool_1_1ReturnToPool__Deleter_1a98a966ad5f76a10937c2da185763fa6c)`(std::weak_ptr< `[`SmartObjectPool`](api-kpsr-application.md#classkpsr_1_1SmartObjectPool)`< T, D > * > pool)` | 
`public inline void `[`operator()`](#structkpsr_1_1SmartObjectPool_1_1ReturnToPool__Deleter_1adb660d7ca905d22da448c5a91e5a36ad)`(T * ptr)` | 

## Members

#### `public inline  explicit `[`ReturnToPool_Deleter`](#structkpsr_1_1SmartObjectPool_1_1ReturnToPool__Deleter_1a98a966ad5f76a10937c2da185763fa6c)`(std::weak_ptr< `[`SmartObjectPool`](api-kpsr-application.md#classkpsr_1_1SmartObjectPool)`< T, D > * > pool)` 

#### `public inline void `[`operator()`](#structkpsr_1_1SmartObjectPool_1_1ReturnToPool__Deleter_1adb660d7ca905d22da448c5a91e5a36ad)`(T * ptr)` 

