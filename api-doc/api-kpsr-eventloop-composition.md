<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-eventloop-composition` 

This group of classes relates exclusively to the assemblying of the application using the eventloop. In Spring terms, the 'wiring' of the application is done using this API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::high_performance::EventLoop`](#classkpsr_1_1high__performance_1_1EventLoop) | The [EventLoop](#classkpsr_1_1high__performance_1_1EventLoop) class.
`class `[`kpsr::high_performance::EventLoopMiddlewareProvider`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider) | The [EventLoopMiddlewareProvider](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider) class.
`class `[`kpsr::high_performance::EventLoopPublisher`](#classkpsr_1_1high__performance_1_1EventLoopPublisher) | The [EventLoopPublisher](#classkpsr_1_1high__performance_1_1EventLoopPublisher) class.
`class `[`kpsr::high_performance::EventLoopSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopSubscriber) | The [EventLoopSubscriber](#classkpsr_1_1high__performance_1_1EventLoopSubscriber) class.

# class `kpsr::high_performance::EventLoop` 

The [EventLoop](#classkpsr_1_1high__performance_1_1EventLoop) class.

Klepsydra Technologies 2019-2020.

2.0.1

This class is not really used by the client code, but it is documented due it its importance in the event loop implementation in Klepsydra. This class basically combines the event emmiter with the high_performance pattern into an event loop with similar behaviour as other event loops implementations. It is as single consumer and multiple publisher ring buffer wrapper with an event emitter API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventLoop`](#classkpsr_1_1high__performance_1_1EventLoop_1a70e617bc76c4e0da57ba7c070459f979)`(`[`kpsr::EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` | [EventLoop](#classkpsr_1_1high__performance_1_1EventLoop).
`public inline void `[`start`](#classkpsr_1_1high__performance_1_1EventLoop_1a215e36a5dcda800896011150c1eceb0b)`()` | start start consumer thread
`public inline void `[`stop`](#classkpsr_1_1high__performance_1_1EventLoop_1a312c6774e5e9faab007c690302826abc)`()` | stop stop consumer thread.
`public inline bool `[`isStarted`](#classkpsr_1_1high__performance_1_1EventLoop_1a78f5201d3e86f5864793feb00f6964bc)`()` | 
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoop_1a8ce3b3c4dfe0216519900065e5877c96) | 
`typedef `[`BatchProcessor`](#classkpsr_1_1high__performance_1_1EventLoop_1a40bf85c842ca09846b0f848167fa47fe) | 

## Members

#### `public inline  `[`EventLoop`](#classkpsr_1_1high__performance_1_1EventLoop_1a70e617bc76c4e0da57ba7c070459f979)`(`[`kpsr::EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` 

[EventLoop](#classkpsr_1_1high__performance_1_1EventLoop).

#### Parameters
* `eventEmitter` 

* `ringBuffer`

#### `public inline void `[`start`](#classkpsr_1_1high__performance_1_1EventLoop_1a215e36a5dcda800896011150c1eceb0b)`()` 

start start consumer thread

#### `public inline void `[`stop`](#classkpsr_1_1high__performance_1_1EventLoop_1a312c6774e5e9faab007c690302826abc)`()` 

stop stop consumer thread.

#### `public inline bool `[`isStarted`](#classkpsr_1_1high__performance_1_1EventLoop_1a78f5201d3e86f5864793feb00f6964bc)`()` 

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoop_1a8ce3b3c4dfe0216519900065e5877c96) 

#### `typedef `[`BatchProcessor`](#classkpsr_1_1high__performance_1_1EventLoop_1a40bf85c842ca09846b0f848167fa47fe) 

# class `kpsr::high_performance::EventLoopMiddlewareProvider` 

The [EventLoopMiddlewareProvider](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider) class.

Klepsydra Technologies 2019-2020.

2.0.1

Main eventloop wizard for creation of event loop and the associated pub/sub pairs. Its used is very straightfoward as shown in this example:

```cpp
// create the provider. There can be as many as need per application.
kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
provider.start();

// retrieve a subscriber, notice the templating API;
kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");

// retrieve a publisher, notice the templating API;
kpsr::Publisher<ELTestEvent> * publisher = provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);
```

The eventloop provider includes a new API for placing function or services in the eventloop as a singlethread scheduler. It is similar to the javascript event loop in this sense.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventLoopMiddlewareProvider`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a0d60cf6a8b2b2df6a5b32a9df16309e2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` | [EventLoopMiddlewareProvider](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider).
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a0b935b98c6e22ff5e440c50f19857505)`(std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` | getPublisher retrieve an object pool based publisher associated to the event loop.
`public template<>`  <br/>`inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1aa95f51fb3d761f8c6012e106cc79b1c9)`(std::string eventName)` | getSubscriber
`public inline `[`Scheduler`](api-undefined.md#classkpsr_1_1Scheduler)` * `[`getScheduler`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1abec5e70df0b72d1d50dd0ad049f605fc)`(std::string name)` | place a function into the event loop. It can be placed for once or repeated execution.
`public inline void `[`start`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a66ead76910146054623a97c455afbbf3)`()` | 
`public inline void `[`stop`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1ac9d0e057a50f9bb0d6607b9b3adb7a17)`()` | 
`public inline bool `[`isRunning`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a10d474e7926d8c98da9c16b1e07a3fa7)`()` | 
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a840960387baa4d9c5dbf90f8cb63a684) | 

## Members

#### `public inline  `[`EventLoopMiddlewareProvider`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a0d60cf6a8b2b2df6a5b32a9df16309e2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` 

[EventLoopMiddlewareProvider](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider).

#### Parameters
* `container`

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a0b935b98c6e22ff5e440c50f19857505)`(std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` 

getPublisher retrieve an object pool based publisher associated to the event loop.

#### Parameters
* `eventName` 

* `poolSize` 

* `initializerFunction` 

* `eventCloner` 

#### Returns

#### `public template<>`  <br/>`inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1aa95f51fb3d761f8c6012e106cc79b1c9)`(std::string eventName)` 

getSubscriber

#### Parameters
* `eventName` 

#### Returns

#### `public inline `[`Scheduler`](api-undefined.md#classkpsr_1_1Scheduler)` * `[`getScheduler`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1abec5e70df0b72d1d50dd0ad049f605fc)`(std::string name)` 

place a function into the event loop. It can be placed for once or repeated execution.

#### Returns

#### `public inline void `[`start`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a66ead76910146054623a97c455afbbf3)`()` 

#### `public inline void `[`stop`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1ac9d0e057a50f9bb0d6607b9b3adb7a17)`()` 

#### `public inline bool `[`isRunning`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a10d474e7926d8c98da9c16b1e07a3fa7)`()` 

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoopMiddlewareProvider_1a840960387baa4d9c5dbf90f8cb63a684) 

# class `kpsr::high_performance::EventLoopPublisher` 

```
class kpsr::high_performance::EventLoopPublisher
  : public kpsr::ObjectPoolPublisher< T >
```  

The [EventLoopPublisher](#classkpsr_1_1high__performance_1_1EventLoopPublisher) class.

Klepsydra Technologies 2019-2020.

2.0.1

Although this function is not really used by the client code directly, it is documented due to its close relation with the event loop. The [EventLoopPublisher](#classkpsr_1_1high__performance_1_1EventLoopPublisher) extends from the [ObjectPoolPublisher](api-kpsr-composition.md#classkpsr_1_1ObjectPoolPublisher) and therefore can handle pools of object in order to improve performance. The tuning can be done via the provider.

The publisher also adds a timestamp to the event when placed in the rinbuffer, so the performance monitor can check how long the message was enqueued before proceed.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public long long unsigned int `[`_discardedMessages`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1afda046194addd0ef6078faa4e233fff9) | _discardedMessages message that could not be place in the ring buffer due to reached capacity
`public inline  `[`EventLoopPublisher`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1a5bbdf09fb231ada8684b351a942e0af2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer,std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` | [EventLoopPublisher](#classkpsr_1_1high__performance_1_1EventLoopPublisher).
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1a33f8e6ed615a6ae4f04e8df8a6dbbd2b) | 

## Members

#### `public long long unsigned int `[`_discardedMessages`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1afda046194addd0ef6078faa4e233fff9) 

_discardedMessages message that could not be place in the ring buffer due to reached capacity

#### `public inline  `[`EventLoopPublisher`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1a5bbdf09fb231ada8684b351a942e0af2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer,std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` 

[EventLoopPublisher](#classkpsr_1_1high__performance_1_1EventLoopPublisher).

#### Parameters
* `container` 

* `ringBuffer` 

* `eventName` 

* `poolSize` 

* `initializerFunction` 

* `eventCloner`

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1EventLoopPublisher_1a33f8e6ed615a6ae4f04e8df8a6dbbd2b) 

# class `kpsr::high_performance::EventLoopSubscriber` 

```
class kpsr::high_performance::EventLoopSubscriber
  : public kpsr::EventEmitterSubscriber< T >
```  

The [EventLoopSubscriber](#classkpsr_1_1high__performance_1_1EventLoopSubscriber) class.

Klepsydra Technologies 2019-2020.

2.0.1

Although this function is not really used by the client code directly, it is documented due to its close relation with the event loop. The [EventLoopPublisher](api-kpsr-eventloop-composition.md#classkpsr_1_1high__performance_1_1EventLoopPublisher) extends from the [EventEmitterSubscriber](api-kpsr-composition.md#classkpsr_1_1EventEmitterSubscriber) and it is contains a map of nested [EventEmitterSubscriber](api-kpsr-composition.md#classkpsr_1_1EventEmitterSubscriber) one per pub/sub.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventLoopSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopSubscriber_1a40ef4aa6d3c7495e6ff7bcc16a9bea1e)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName)` | [EventLoopSubscriber](#classkpsr_1_1high__performance_1_1EventLoopSubscriber).
`public inline  `[`~EventLoopSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopSubscriber_1a2374e8266070c997e1bcaeddbaca9e70)`()` | 

## Members

#### `public inline  `[`EventLoopSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopSubscriber_1a40ef4aa6d3c7495e6ff7bcc16a9bea1e)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName)` 

[EventLoopSubscriber](#classkpsr_1_1high__performance_1_1EventLoopSubscriber).

#### Parameters
* `container` 

* `eventEmitter` 

* `eventName`

#### `public inline  `[`~EventLoopSubscriber`](#classkpsr_1_1high__performance_1_1EventLoopSubscriber_1a2374e8266070c997e1bcaeddbaca9e70)`()` 

