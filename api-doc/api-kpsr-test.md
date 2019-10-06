<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-test` 

This group of classes are meant to be used in the unit testing of the application. They provide facilities to test middleware publishing and subscription along with some other syntatic sugar for testing.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::mem::CacheListener`](#classkpsr_1_1mem_1_1CacheListener) | The [CacheListener](#classkpsr_1_1mem_1_1CacheListener) class.
`class `[`kpsr::mem::MultiThreadCacheListener`](#classkpsr_1_1mem_1_1MultiThreadCacheListener) | The [MultiThreadCacheListener](#classkpsr_1_1mem_1_1MultiThreadCacheListener) class.
`class `[`kpsr::mem::TestCacheListener`](#classkpsr_1_1mem_1_1TestCacheListener) | The [TestCacheListener](#classkpsr_1_1mem_1_1TestCacheListener) class.
`class `[`kpsr::EventEmitterMiddlewareProvider`](#classkpsr_1_1EventEmitterMiddlewareProvider) | The [EventEmitterMiddlewareProvider](#classkpsr_1_1EventEmitterMiddlewareProvider) class.
`class `[`kpsr::EventEmitterPublisher`](#classkpsr_1_1EventEmitterPublisher) | The [EventEmitterPublisher](#classkpsr_1_1EventEmitterPublisher) class.

# class `kpsr::mem::CacheListener` 

The [CacheListener](#classkpsr_1_1mem_1_1CacheListener) class.

Klepsydra Technologies 2019-2020.

2.0.1

This class is part of the API facility framework build in Klepsydra. The [CacheListener](#classkpsr_1_1mem_1_1CacheListener) stores a copy of the last received event. It offers also an std::function that can be plugged directly into the subscriber. For example: 
```cpp
Subscriber<std::string> subscriber;
CacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
```
 Finally, it also keeps count of the number of event received and total time spent in copying.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::function< void(const T &)> `[`cacheListenerFunction`](#classkpsr_1_1mem_1_1CacheListener_1a0e5a94bebf046582352a47efd5a8f2fc) | cacheListenerFunction
`public std::atomic_int `[`counter`](#classkpsr_1_1mem_1_1CacheListener_1a0985955a93be97ef86a734b8c63915a0) | counter
`public long `[`totalCopyingTime`](#classkpsr_1_1mem_1_1CacheListener_1a99877d490c8c9cc19b6b7143f1efb96a) | totalCopyingTime
`public inline  `[`CacheListener`](#classkpsr_1_1mem_1_1CacheListener_1a4ea10ca2dd964cf29ab99dae695ffc2e)`()` | [CacheListener](#classkpsr_1_1mem_1_1CacheListener).
`public inline virtual  `[`~CacheListener`](#classkpsr_1_1mem_1_1CacheListener_1afe0d931f999918fc2b7f1d4081194f10)`()` | 
`public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1CacheListener_1a2c9fd91387b1231f38c4214e7daad927)`(const T & event)` | onEventReceived
`public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1CacheListener_1adcf4fc3509fca45506bc87da4d571eec)`(std::shared_ptr< T > event)` | onEventReceived
`public inline virtual std::shared_ptr< T > `[`getLastReceivedEvent`](#classkpsr_1_1mem_1_1CacheListener_1ab17de9e5cb5db7607dcc9a3e387b3801)`()` | getLastReceivedEvent

## Members

#### `public std::function< void(const T &)> `[`cacheListenerFunction`](#classkpsr_1_1mem_1_1CacheListener_1a0e5a94bebf046582352a47efd5a8f2fc) 

cacheListenerFunction

#### `public std::atomic_int `[`counter`](#classkpsr_1_1mem_1_1CacheListener_1a0985955a93be97ef86a734b8c63915a0) 

counter

#### `public long `[`totalCopyingTime`](#classkpsr_1_1mem_1_1CacheListener_1a99877d490c8c9cc19b6b7143f1efb96a) 

totalCopyingTime

#### `public inline  `[`CacheListener`](#classkpsr_1_1mem_1_1CacheListener_1a4ea10ca2dd964cf29ab99dae695ffc2e)`()` 

[CacheListener](#classkpsr_1_1mem_1_1CacheListener).

#### Parameters
* `sleepTimeMs` When a value larger than 0 is passed, it will sleep after copying the event.

#### `public inline virtual  `[`~CacheListener`](#classkpsr_1_1mem_1_1CacheListener_1afe0d931f999918fc2b7f1d4081194f10)`()` 

#### `public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1CacheListener_1a2c9fd91387b1231f38c4214e7daad927)`(const T & event)` 

onEventReceived

#### Parameters
* `event`

#### `public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1CacheListener_1adcf4fc3509fca45506bc87da4d571eec)`(std::shared_ptr< T > event)` 

onEventReceived

#### Parameters
* `event`

#### `public inline virtual std::shared_ptr< T > `[`getLastReceivedEvent`](#classkpsr_1_1mem_1_1CacheListener_1ab17de9e5cb5db7607dcc9a3e387b3801)`()` 

getLastReceivedEvent

#### Returns

# class `kpsr::mem::MultiThreadCacheListener` 

```
class kpsr::mem::MultiThreadCacheListener
  : public kpsr::mem::CacheListener< T >
```  

The [MultiThreadCacheListener](#classkpsr_1_1mem_1_1MultiThreadCacheListener) class.

Klepsydra Technologies 2019-2020.

2.0.1

This class is part of the API facility framework build in Klepsydra. The [MultiThreadCacheListener](#classkpsr_1_1mem_1_1MultiThreadCacheListener) stores a copy of the last received event in a thread-safe manner. It offers also an std::function that can be plugged directly into the subscriber. For example: 
```cpp
Subscriber<std::string> subscriber;
MultiThreadCacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`MultiThreadCacheListener`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a43649f27a0cc6f4cbed658864b899503)`()` | [CacheListener](api-kpsr-test.md#classkpsr_1_1mem_1_1CacheListener).
`public inline virtual  `[`~MultiThreadCacheListener`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a94e11172d8f454bb43a8dc6f7bc9cb42)`()` | 
`public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a97f4b9336b67da613be738cf49eb6cb7)`(const T & event)` | onEventReceived
`public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1ae8a1629aca87282a184f2510a6314446)`(std::shared_ptr< T > event)` | onEventReceived
`public inline virtual std::shared_ptr< T > `[`getLastReceivedEvent`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1af7d9e2c42f7b193e39c17c2c3f83e9f8)`()` | getLastReceivedEvent
`public inline std::pair< int, std::shared_ptr< T > > `[`getCounterAndEvent`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1ac9a71297aeafabc1c9cc2a4b62623820)`()` | getCounterAndEvent

## Members

#### `public inline  `[`MultiThreadCacheListener`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a43649f27a0cc6f4cbed658864b899503)`()` 

[CacheListener](api-kpsr-test.md#classkpsr_1_1mem_1_1CacheListener).

#### Parameters
* `sleepTimeMs` When a value larger than 0 is passed, it will sleep after copying the event.

#### `public inline virtual  `[`~MultiThreadCacheListener`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a94e11172d8f454bb43a8dc6f7bc9cb42)`()` 

#### `public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1a97f4b9336b67da613be738cf49eb6cb7)`(const T & event)` 

onEventReceived

#### Parameters
* `event`

#### `public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1ae8a1629aca87282a184f2510a6314446)`(std::shared_ptr< T > event)` 

onEventReceived

#### Parameters
* `event`

#### `public inline virtual std::shared_ptr< T > `[`getLastReceivedEvent`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1af7d9e2c42f7b193e39c17c2c3f83e9f8)`()` 

getLastReceivedEvent

#### Returns

#### `public inline std::pair< int, std::shared_ptr< T > > `[`getCounterAndEvent`](#classkpsr_1_1mem_1_1MultiThreadCacheListener_1ac9a71297aeafabc1c9cc2a4b62623820)`()` 

getCounterAndEvent

#### Returns

# class `kpsr::mem::TestCacheListener` 

```
class kpsr::mem::TestCacheListener
  : public kpsr::mem::CacheListener< T >
```  

The [TestCacheListener](#classkpsr_1_1mem_1_1TestCacheListener) class.

Klepsydra Technologies 2019-2020.

2.0.1

This class is part of the test facility framework build in Klepsydra. The [CacheListener](api-kpsr-test.md#classkpsr_1_1mem_1_1CacheListener) stores a copy of the last received event. It offers also an std::function that can be plugged directly into the subscriber. For example: 
```cpp
Subscriber<std::string> subscriber;
TestCacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
```
 It can also sleep for a configured number of milliseconds as a way to mock processing time. Finally, it also keeps count of the number of event received and total time spent in copying.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`TestCacheListener`](#classkpsr_1_1mem_1_1TestCacheListener_1a23ef7a0a0d4dbff8e658e4ae3d1c57b4)`(int sleepTimeMs)` | [CacheListener](api-kpsr-test.md#classkpsr_1_1mem_1_1CacheListener).
`public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1TestCacheListener_1accec73969e8b72cbba99299629070911)`(const T & event)` | onEventReceived
`public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1TestCacheListener_1a66730b08de85cb0af49a29533b9eea17)`(std::shared_ptr< T > event)` | onEventReceived

## Members

#### `public inline  `[`TestCacheListener`](#classkpsr_1_1mem_1_1TestCacheListener_1a23ef7a0a0d4dbff8e658e4ae3d1c57b4)`(int sleepTimeMs)` 

[CacheListener](api-kpsr-test.md#classkpsr_1_1mem_1_1CacheListener).

#### Parameters
* `sleepTimeMs` When a value larger than 0 is passed, it will sleep after copying the event.

#### `public inline virtual void `[`onEventReceived`](#classkpsr_1_1mem_1_1TestCacheListener_1accec73969e8b72cbba99299629070911)`(const T & event)` 

onEventReceived

#### Parameters
* `event`

#### `public inline virtual void `[`onEventReceivedRaw`](#classkpsr_1_1mem_1_1TestCacheListener_1a66730b08de85cb0af49a29533b9eea17)`(std::shared_ptr< T > event)` 

onEventReceived

#### Parameters
* `event`

# class `kpsr::EventEmitterMiddlewareProvider` 

The [EventEmitterMiddlewareProvider](#classkpsr_1_1EventEmitterMiddlewareProvider) class.

Klepsydra Technologies 2019-2020.

2.1.0

This class a is facility wizard to create pub-sub pairs. It keeps tracks of them in a map. This particular implementation of a provider is intenteded for testing purposes only.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventEmitterMiddlewareProvider`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a3ac6e0d051b866c84dfdfa6900eb1c85)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` | [EventEmitterMiddlewareProvider](#classkpsr_1_1EventEmitterMiddlewareProvider).
`public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a82b582c0574eb53181640e9d732c7e73)`()` | getPublisher
`public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a6ac1a7c2a19242e4e59224abc2b662a7)`()` | getSubscriber
`public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< S, T > > `[`getProcessForwarder`](#classkpsr_1_1EventEmitterMiddlewareProvider_1add50e46a74bea44ded6f488d697a83ab)`(const std::function< void(const S &, T &)> & transformFunction)` | getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.

## Members

#### `public inline  `[`EventEmitterMiddlewareProvider`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a3ac6e0d051b866c84dfdfa6900eb1c85)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` 

[EventEmitterMiddlewareProvider](#classkpsr_1_1EventEmitterMiddlewareProvider).

#### Parameters
* `container` 

* `eventName` 

* `poolSize` 

* `initializerFunction` 

* `eventCloner`

#### `public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a82b582c0574eb53181640e9d732c7e73)`()` 

getPublisher

#### Returns

#### `public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1EventEmitterMiddlewareProvider_1a6ac1a7c2a19242e4e59224abc2b662a7)`()` 

getSubscriber

#### Returns

#### `public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< S, T > > `[`getProcessForwarder`](#classkpsr_1_1EventEmitterMiddlewareProvider_1add50e46a74bea44ded6f488d697a83ab)`(const std::function< void(const S &, T &)> & transformFunction)` 

getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.

#### Parameters
* `transformFunction` 

#### Returns

# class `kpsr::EventEmitterPublisher` 

```
class kpsr::EventEmitterPublisher
  : public kpsr::ObjectPoolPublisher< T >
```  

The [EventEmitterPublisher](#classkpsr_1_1EventEmitterPublisher) class.

Klepsydra Technologies 2019-2020.

2.1.0

This publisher implementation is intended for testing purposes. It is synchronous and single-threaded. The goal is to use this publisher as a testing publisher for unit and integration testing. It derives from the [ObjectPoolPublisher](api-kpsr-composition.md#classkpsr_1_1ObjectPoolPublisher), which allows to pre-allocate the message instances in a pool for increasing the performance.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventEmitterPublisher`](#classkpsr_1_1EventEmitterPublisher_1a5fe73e3d56e0df83f03b6918f457044a)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` | [EventEmitterPublisher](#classkpsr_1_1EventEmitterPublisher).

## Members

#### `public inline  `[`EventEmitterPublisher`](#classkpsr_1_1EventEmitterPublisher_1a5fe73e3d56e0df83f03b6918f457044a)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` 

[EventEmitterPublisher](#classkpsr_1_1EventEmitterPublisher).

#### Parameters
* `container` 

* `eventName` 

* `eventEmitter` 

* `poolSize` 

* `initializerFunction` 

* `eventCloner`

