<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-mem-composition` 

This group of classes relates exclusively to the assemblying of the application for intra and inter process comm. In Spring terms, the 'wiring' of the application is done using this API. This API can be used in conjunction with the DDS ROS and ZMQ middleware to increase peformance and stability.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::mem::BasicMiddlewareProvider`](#classkpsr_1_1mem_1_1BasicMiddlewareProvider) | The [BasicMiddlewareProvider](#classkpsr_1_1mem_1_1BasicMiddlewareProvider) class.
`class `[`kpsr::mem::BasicPublisher`](#classkpsr_1_1mem_1_1BasicPublisher) | The [BasicPublisher](#classkpsr_1_1mem_1_1BasicPublisher) class.
`class `[`kpsr::mem::ConcurrentMiddlewareProvider`](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider) | The [ConcurrentMiddlewareProvider](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider) class.
`class `[`kpsr::mem::ConcurrentQueuePoller`](#classkpsr_1_1mem_1_1ConcurrentQueuePoller) | The [ConcurrentQueuePoller](#classkpsr_1_1mem_1_1ConcurrentQueuePoller) class.
`class `[`kpsr::mem::ConcurrentQueuePublisher`](#classkpsr_1_1mem_1_1ConcurrentQueuePublisher) | The ConcurrentPublisher class.
`class `[`kpsr::mem::InMemoryMiddlewareProvider`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider) | The [InMemoryMiddlewareProvider](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider) class.
`class `[`kpsr::mem::InMemoryQueuePoller`](#classkpsr_1_1mem_1_1InMemoryQueuePoller) | The [InMemoryQueuePoller](#classkpsr_1_1mem_1_1InMemoryQueuePoller) class.
`class `[`kpsr::mem::SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue) | The [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue) class.
`class `[`kpsr::mem::SafeQueuePoller`](#classkpsr_1_1mem_1_1SafeQueuePoller) | The [SafeQueuePoller](#classkpsr_1_1mem_1_1SafeQueuePoller) class.
`struct `[`kpsr::mem::EventData`](#structkpsr_1_1mem_1_1EventData) | The [EventData](#structkpsr_1_1mem_1_1EventData) struct.

# class `kpsr::mem::BasicMiddlewareProvider` 

```
class kpsr::mem::BasicMiddlewareProvider
  : public kpsr::mem::InMemoryMiddlewareProvider< T >
```  

The [BasicMiddlewareProvider](#classkpsr_1_1mem_1_1BasicMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

This class is a wizard that creates the safequeue, publishers and subscriber in combination with some memory management functions.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`BasicMiddlewareProvider`](#classkpsr_1_1mem_1_1BasicMiddlewareProvider_1a6949218658280042d55b10c27e77757b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int queueSize,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,bool discardItemsWhenFull)` | [BasicMiddlewareProvider](#classkpsr_1_1mem_1_1BasicMiddlewareProvider).

## Members

#### `public inline  `[`BasicMiddlewareProvider`](#classkpsr_1_1mem_1_1BasicMiddlewareProvider_1a6949218658280042d55b10c27e77757b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int queueSize,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,bool discardItemsWhenFull)` 

[BasicMiddlewareProvider](#classkpsr_1_1mem_1_1BasicMiddlewareProvider).

#### Parameters
* `container` 

* `eventName` 

* `queueSize` 

* `poolSize` 

* `initializerFunction` function to be invoked after event instantiaion. 

* `eventCloner` a function used to clone events after copied in the publish method. 

* `discardItemsWhenFull` when true, old events will be deleted when the queue is full and new one need to be put. In the false case, the publisher will block until there is free space to put new events in the queue, if the queue is full.

# class `kpsr::mem::BasicPublisher` 

```
class kpsr::mem::BasicPublisher
  : public kpsr::ObjectPoolPublisher< T >
```  

The [BasicPublisher](#classkpsr_1_1mem_1_1BasicPublisher) class.

2023 Klepsydra Technologies AG

2.1.0

Publishing class that puts events in the safequeue. It has additional configuration to optimise memory allocation.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`BasicPublisher`](#classkpsr_1_1mem_1_1BasicPublisher_1a989c263dd9d83ea76fd8e749179a3de2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,`[`SafeQueue](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue)< [EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,bool discardItemsWhenFull)` | [BasicPublisher](#classkpsr_1_1mem_1_1BasicPublisher).

## Members

#### `public inline  `[`BasicPublisher`](#classkpsr_1_1mem_1_1BasicPublisher_1a989c263dd9d83ea76fd8e749179a3de2)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,`[`SafeQueue](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue)< [EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,bool discardItemsWhenFull)` 

[BasicPublisher](#classkpsr_1_1mem_1_1BasicPublisher).

#### Parameters
* `container` 

* `name` 

* `poolSize` 

* `initializerFunction` function to be invoked after event instantiaion. 

* `eventCloner` a function used to clone events after copied in the publish method. 

* `safeQueue` 

* `discardItemsWhenFull` when true, old events will be deleted when the queue is full and new one need to be put. In the false case, the publisher will block until there is free space to put new events in the queue, if the queue is full.

# class `kpsr::mem::ConcurrentMiddlewareProvider` 

```
class kpsr::mem::ConcurrentMiddlewareProvider
  : public kpsr::mem::InMemoryMiddlewareProvider< T >
```  

The [ConcurrentMiddlewareProvider](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

This class is a wizard that creates the safequeue (using the lock free concurrent queue), publishers and subscriber in combination with some memory management functions.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ConcurrentMiddlewareProvider`](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider_1acee1288456ecc613b496fe5e9906f12b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int queueSize,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,bool discardItemsWhenFull,unsigned int sleepPeriodUS)` | [ConcurrentMiddlewareProvider](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider).

## Members

#### `public inline  `[`ConcurrentMiddlewareProvider`](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider_1acee1288456ecc613b496fe5e9906f12b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName,int queueSize,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,bool discardItemsWhenFull,unsigned int sleepPeriodUS)` 

[ConcurrentMiddlewareProvider](#classkpsr_1_1mem_1_1ConcurrentMiddlewareProvider).

#### Parameters
* `container` 

* `eventName` 

* `queueSize` 

* `poolSize` 

* `initializerFunction` function to be invoked after event instantiaion. 

* `eventCloner` a function used to clone events after copied in the publish method. 

* `discardItemsWhenFull` when true, old events will be deleted when the queue is full and new one need to be put. In the false case, the publisher will block until there is free space to put new events in the queue, if the queue is full.

# class `kpsr::mem::ConcurrentQueuePoller` 

```
class kpsr::mem::ConcurrentQueuePoller
  : public kpsr::mem::InMemoryQueuePoller
```  

The [ConcurrentQueuePoller](#classkpsr_1_1mem_1_1ConcurrentQueuePoller) class.

2023 Klepsydra Technologies AG

2.1.0

This class, which extends from the [event_emitter_subscriber.h](#event__emitter__subscriber_8h_source) is an asynchronous in-memory middleware. It blocks the subcriber thread when no events are available. This class uses the non-locking concurrent queue.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ConcurrentQueuePoller`](#classkpsr_1_1mem_1_1ConcurrentQueuePoller_1ac83bb44a6d590cce4c03dd66bd888695)`(moodycamel::ConcurrentQueue< `[`EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & concurrentQueue,`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS,moodycamel::ProducerToken & token)` | [ConcurrentQueuePoller](#classkpsr_1_1mem_1_1ConcurrentQueuePoller).

## Members

#### `public inline  `[`ConcurrentQueuePoller`](#classkpsr_1_1mem_1_1ConcurrentQueuePoller_1ac83bb44a6d590cce4c03dd66bd888695)`(moodycamel::ConcurrentQueue< `[`EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & concurrentQueue,`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS,moodycamel::ProducerToken & token)` 

[ConcurrentQueuePoller](#classkpsr_1_1mem_1_1ConcurrentQueuePoller).

#### Parameters
* `concurrentQueue` 

* `eventEmitter` 

* `eventName` 

* `sleepPeriodUS` The time in microseconds to sleep/wait 

* `token` The producer token used by the publisher

# class `kpsr::mem::ConcurrentQueuePublisher` 

```
class kpsr::mem::ConcurrentQueuePublisher
  : public kpsr::ObjectPoolPublisher< T >
```  

The ConcurrentPublisher class.

2023 Klepsydra Technologies AG

2.1.0

Publishing class that puts events in the safequeue. It has additional configuration to optimise memory allocation.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ConcurrentQueuePublisher`](#classkpsr_1_1mem_1_1ConcurrentQueuePublisher_1a2325ca45db27ef0029124550ad9d408b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,moodycamel::ConcurrentQueue< `[`EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,bool discardItemsWhenFull,moodycamel::ProducerToken & token)` | [ConcurrentQueuePublisher](#classkpsr_1_1mem_1_1ConcurrentQueuePublisher).

## Members

#### `public inline  `[`ConcurrentQueuePublisher`](#classkpsr_1_1mem_1_1ConcurrentQueuePublisher_1a2325ca45db27ef0029124550ad9d408b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner,moodycamel::ConcurrentQueue< `[`EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,bool discardItemsWhenFull,moodycamel::ProducerToken & token)` 

[ConcurrentQueuePublisher](#classkpsr_1_1mem_1_1ConcurrentQueuePublisher).

#### Parameters
* `container` 

* `name` 

* `poolSize` 

* `initializerFunction` function to be invoked after event instantiaion. 

* `eventCloner` a function used to clone events after copied in the publish method. 

* `safeQueue` 

* `discardItemsWhenFull` when true, old events will be deleted when the queue is full and new one need to be put. In the false case, the publisher will block until there is free space to put new events in the queue, if the queue is full.

# class `kpsr::mem::InMemoryMiddlewareProvider` 

The [InMemoryMiddlewareProvider](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

This class is an abstract class that provides the API for the wizard that creates the internal queue, publishers and subscriber in combination with some memory management functions.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public `[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` `[`_eventEmitter`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1aedc008d2958d183cf32a7312aadfb278) | _eventEmitter
`public std::string `[`_eventName`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1abb53466b864118de277e0c57f1f1c253) | _eventName
`public inline  `[`InMemoryMiddlewareProvider`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a61de11267377a04607cb8ed04f93ec64)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName)` | [InMemoryMiddlewareProvider](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider).
`public inline virtual void `[`start`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a2a5cedc727a72151c420f9f878f1a40b)`()` | start
`public inline virtual void `[`stop`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1af5a1f1a13efad8552c8df9d25b0212fa)`()` | stop
`public inline virtual bool `[`isRunning`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a6f5c6aa50e2787e295817d03fa214fb8)`()` | isRunning
`public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1ac0a34a38593c2d7b31f7c59dfa833bd1)`()` | getPublisher
`public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1abcb11afa971b634564f806730bcbf284)`()` | getSubscriber
`public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< S, T > > `[`getProcessForwarder`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1ad700016fbee653647f6e34390d78a80c)`(const std::function< void(const S &, T &)> & transformFunction)` | getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.
`protected `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`_publisher`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a82bbebc561fea3779b262e725a3bfeae) | 
`protected `[`InMemoryQueuePoller`](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1InMemoryQueuePoller)` * `[`_poller`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a5f503ff37f0f08744142402ef1efc9e4) | 
`protected `[`EventEmitterSubscriber`](api-kpsr-composition.md#classkpsr_1_1EventEmitterSubscriber)`< T > `[`_subscriber`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a3f7fa24d32c301afa4e0a2c44d86bb15) | 

## Members

#### `public `[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` `[`_eventEmitter`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1aedc008d2958d183cf32a7312aadfb278) 

_eventEmitter

#### `public std::string `[`_eventName`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1abb53466b864118de277e0c57f1f1c253) 

_eventName

#### `public inline  `[`InMemoryMiddlewareProvider`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a61de11267377a04607cb8ed04f93ec64)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string eventName)` 

[InMemoryMiddlewareProvider](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider).

#### Parameters
* `container` 

* `eventName` 

* `poolSize`

#### `public inline virtual void `[`start`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a2a5cedc727a72151c420f9f878f1a40b)`()` 

start

#### `public inline virtual void `[`stop`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1af5a1f1a13efad8552c8df9d25b0212fa)`()` 

stop

#### `public inline virtual bool `[`isRunning`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a6f5c6aa50e2787e295817d03fa214fb8)`()` 

isRunning

#### Returns

#### `public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getPublisher`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1ac0a34a38593c2d7b31f7c59dfa833bd1)`()` 

getPublisher

#### Returns

#### `public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< T > * `[`getSubscriber`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1abcb11afa971b634564f806730bcbf284)`()` 

getSubscriber

#### Returns

#### `public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< S, T > > `[`getProcessForwarder`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1ad700016fbee653647f6e34390d78a80c)`(const std::function< void(const S &, T &)> & transformFunction)` 

getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.

#### Parameters
* `transformFunction` 

#### Returns

#### `protected `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`_publisher`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a82bbebc561fea3779b262e725a3bfeae) 

#### `protected `[`InMemoryQueuePoller`](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1InMemoryQueuePoller)` * `[`_poller`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a5f503ff37f0f08744142402ef1efc9e4) 

#### `protected `[`EventEmitterSubscriber`](api-kpsr-composition.md#classkpsr_1_1EventEmitterSubscriber)`< T > `[`_subscriber`](#classkpsr_1_1mem_1_1InMemoryMiddlewareProvider_1a3f7fa24d32c301afa4e0a2c44d86bb15) 

# class `kpsr::mem::InMemoryQueuePoller` 

The [InMemoryQueuePoller](#classkpsr_1_1mem_1_1InMemoryQueuePoller) class.

2023 Klepsydra Technologies AG

2.1.0

This class, provides the API to a class which extends from the [event_emitter_subscriber.h](#event__emitter__subscriber_8h_source) is an asynchronous in-memory middleware. It block the subcriber thread when no events are availble. The polling loop depends on the type of queue being used (locking or non-locking).

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::atomic< bool > `[`_running`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1afaf3606fe38bce7e5659c5e2260dc830) | _running
`public inline  `[`InMemoryQueuePoller`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6510ec9105023b5eaea6400beef03676)`(`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS)` | [InMemoryQueuePoller](#classkpsr_1_1mem_1_1InMemoryQueuePoller).
`public void `[`start`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6a3642bf61f3ebe08e356f07ec7deae9)`()` | start
`public void `[`stop`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1ae602aefce06ef43e60c9b329a7c11b90)`()` | stop
`public  `[`~InMemoryQueuePoller`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6aac1c7004f29eb68164a8e1425a0285)`()` | 
`protected `[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & `[`_eventEmitter`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a60228eee37f761399d6508be95bec0ab) | 
`protected std::string `[`_eventName`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a53939321923a6004dd7a04ea51e27a87) | 
`protected std::thread `[`_threadNotifier`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a8181e976a68bc64cf298445aed290008) | 
`protected unsigned int `[`_sleepPeriodUS`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a1b85b3d4def4cdc8208ec80e21bab5dc) | 

## Members

#### `public std::atomic< bool > `[`_running`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1afaf3606fe38bce7e5659c5e2260dc830) 

_running

#### `public inline  `[`InMemoryQueuePoller`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6510ec9105023b5eaea6400beef03676)`(`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS)` 

[InMemoryQueuePoller](#classkpsr_1_1mem_1_1InMemoryQueuePoller).

#### Parameters
* `eventEmitter` 

* `eventName` 

* `sleepPeriodUS` The time in microseconds to sleep/wait

#### `public void `[`start`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6a3642bf61f3ebe08e356f07ec7deae9)`()` 

start

#### `public void `[`stop`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1ae602aefce06ef43e60c9b329a7c11b90)`()` 

stop

#### `public  `[`~InMemoryQueuePoller`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a6aac1c7004f29eb68164a8e1425a0285)`()` 

#### `protected `[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & `[`_eventEmitter`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a60228eee37f761399d6508be95bec0ab) 

#### `protected std::string `[`_eventName`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a53939321923a6004dd7a04ea51e27a87) 

#### `protected std::thread `[`_threadNotifier`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a8181e976a68bc64cf298445aed290008) 

#### `protected unsigned int `[`_sleepPeriodUS`](#classkpsr_1_1mem_1_1InMemoryQueuePoller_1a1b85b3d4def4cdc8208ec80e21bab5dc) 

# class `kpsr::mem::SafeQueue` 

The [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue) class.

2023 Klepsydra Technologies AG

2.1.0

A thread-safe asynchronous blocking queue.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1ad6a2426319773c7521e1427fecdc6744)`(unsigned int max_num_items)` | [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).
`public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1ae61465c1cd394f496de262946cb19253)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` && sq)` | [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).
`public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1aae1af1162b0e4e6737c75eb09d83a7f2)`(const `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` | [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).
`public inline  `[`~SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1a20201ff071c3ce2398c0f2b333864460)`()` | 
`public inline void `[`set_max_num_items`](#classkpsr_1_1mem_1_1SafeQueue_1a4caad18a8c1444d2cbccc75781892416)`(unsigned int max_num_items)` | set_max_num_items Sets the maximum number of items in the queue. Defaults is 0: No limit
`public inline bool `[`push`](#classkpsr_1_1mem_1_1SafeQueue_1ad04d778bf787ea9837d7e225382012b5)`(const value_type & item)` | push Pushes the item into the queue. It blocks if the queue is full
`public inline bool `[`push`](#classkpsr_1_1mem_1_1SafeQueue_1a1270af3bea3209ea604fca162af73182)`(const value_type && item)` | push Pushes the item into the queue. It blocks if the queue is full
`public inline bool `[`move_push`](#classkpsr_1_1mem_1_1SafeQueue_1ad9f7b0981770a4cd17ca1110c4dc77b0)`(const value_type & item)` | move_push Move ownership and pushes the item into the queue. It blocks if the queue is full
`public inline bool `[`try_push`](#classkpsr_1_1mem_1_1SafeQueue_1ae513ab8a110e7841c4da69eb85dd84d7)`(const value_type & item)` | try_push Pushes the item into the queue. Not blocking call.
`public inline bool `[`try_push`](#classkpsr_1_1mem_1_1SafeQueue_1a8bdcaa078fea7c53059e41727c9de435)`(const value_type && item)` | try_push Pushes the item into the queue. Not blocking call.
`public inline bool `[`try_move_push`](#classkpsr_1_1mem_1_1SafeQueue_1a1ce946456cc9e35d59b56ea2dd8bc893)`(const value_type & item)` | try_push Move ownership and pushesthe item into the queue. Not blocking call.
`public inline uint `[`force_move_push`](#classkpsr_1_1mem_1_1SafeQueue_1a64fdd28163fc6cc1ef68931d63ebd6b0)`(const value_type & item)` | force_push Move ownership and pushesthe item into the queue, remove previous items if the queue is full.
`public inline void `[`pop`](#classkpsr_1_1mem_1_1SafeQueue_1abd44052b41a0e87237897849efa5a3fc)`(value_type & item)` | pop Pops item from the queue. If queue is empty, this function blocks until item becomes available.
`public inline void `[`move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1aa5197ae1c039640c717fb52d5de4449c)`(value_type & item)` | move_pop Pops item from the queue using the contained type's move assignment operator, if it has one. This method is identical to the [pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1abd44052b41a0e87237897849efa5a3fc) method if that type has no move assignment operator. If queue is empty, this function blocks until item becomes available.
`public inline bool `[`try_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a)`(value_type & item)` | try_pop Tries to pop item from the queue.
`public inline bool `[`try_move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a4bd633b1d7bbf4b370cff51dceacf93e)`(value_type & item)` | try_move_pop Tries to pop item from the queue using the contained type's move assignment operator, if it has one. This method is identical to the [try_pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a) method if that type has no move assignment operator.
`public inline bool `[`timeout_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a2556d0ab30c2cc84ac74cd13e28422d9)`(value_type & item,std::uint64_t timeout)` | timeout_pop Pops item from the queue. If the queue is empty, blocks for timeout microseconds, or until item becomes available.
`public inline bool `[`timeout_move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a2ecb978a337d0122ad029723a19ac01f)`(value_type & item,std::uint64_t timeout)` | timeout_move_pop Pops item from the queue using the contained type's move assignment operator, if it has one.. If the queue is empty, blocks for timeout microseconds, or until item becomes available. This method is identical to the [try_pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a) method if that type has no move assignment operator.
`public inline size_type `[`size`](#classkpsr_1_1mem_1_1SafeQueue_1a2022ee7832d5c01aa6af8a8da7e08dac)`() const` | size Gets the number of items in the queue.
`public inline bool `[`empty`](#classkpsr_1_1mem_1_1SafeQueue_1a42fc93d83ccb308a3893fb0af5fb593a)`() const` | empty Check if the queue is empty.
`public inline bool `[`full`](#classkpsr_1_1mem_1_1SafeQueue_1ad7e81223f4c2fad7f82e314e3db719a0)`() const` | full Check if the queue is full.
`public inline void `[`swap`](#classkpsr_1_1mem_1_1SafeQueue_1a56b5f32fe0543235506794546d42c9d3)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` | swap Swaps the contents.
`public inline `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & `[`operator=`](#classkpsr_1_1mem_1_1SafeQueue_1a3b849d0a69bd1e730219650dff2c8e10)`(const `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` | operator = The copy assignment operator
`public inline `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & `[`operator=`](#classkpsr_1_1mem_1_1SafeQueue_1a53cf5c9071a8d82906fb6587a84bdc37)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` && sq)` | operator = The move assignment operator

## Members

#### `public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1ad6a2426319773c7521e1427fecdc6744)`(unsigned int max_num_items)` 

[SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).

#### Parameters
* `max_num_items`

#### `public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1ae61465c1cd394f496de262946cb19253)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` && sq)` 

[SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).

#### Parameters
* `sq`

#### `public inline  `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1aae1af1162b0e4e6737c75eb09d83a7f2)`(const `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` 

[SafeQueue](#classkpsr_1_1mem_1_1SafeQueue).

#### Parameters
* `sq`

#### `public inline  `[`~SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue_1a20201ff071c3ce2398c0f2b333864460)`()` 

#### `public inline void `[`set_max_num_items`](#classkpsr_1_1mem_1_1SafeQueue_1a4caad18a8c1444d2cbccc75781892416)`(unsigned int max_num_items)` 

set_max_num_items Sets the maximum number of items in the queue. Defaults is 0: No limit

#### Parameters
* `max_num_items`

#### `public inline bool `[`push`](#classkpsr_1_1mem_1_1SafeQueue_1ad04d778bf787ea9837d7e225382012b5)`(const value_type & item)` 

push Pushes the item into the queue. It blocks if the queue is full

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline bool `[`push`](#classkpsr_1_1mem_1_1SafeQueue_1a1270af3bea3209ea604fca162af73182)`(const value_type && item)` 

push Pushes the item into the queue. It blocks if the queue is full

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline bool `[`move_push`](#classkpsr_1_1mem_1_1SafeQueue_1ad9f7b0981770a4cd17ca1110c4dc77b0)`(const value_type & item)` 

move_push Move ownership and pushes the item into the queue. It blocks if the queue is full

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline bool `[`try_push`](#classkpsr_1_1mem_1_1SafeQueue_1ae513ab8a110e7841c4da69eb85dd84d7)`(const value_type & item)` 

try_push Pushes the item into the queue. Not blocking call.

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline bool `[`try_push`](#classkpsr_1_1mem_1_1SafeQueue_1a8bdcaa078fea7c53059e41727c9de435)`(const value_type && item)` 

try_push Pushes the item into the queue. Not blocking call.

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline bool `[`try_move_push`](#classkpsr_1_1mem_1_1SafeQueue_1a1ce946456cc9e35d59b56ea2dd8bc893)`(const value_type & item)` 

try_push Move ownership and pushesthe item into the queue. Not blocking call.

#### Parameters
* `item` An item. 

#### Returns
true if an item was pushed into the queue

#### `public inline uint `[`force_move_push`](#classkpsr_1_1mem_1_1SafeQueue_1a64fdd28163fc6cc1ef68931d63ebd6b0)`(const value_type & item)` 

force_push Move ownership and pushesthe item into the queue, remove previous items if the queue is full.

#### Parameters
* `item` An item. 

#### Returns
number of items that were removed in order to push the new one.

#### `public inline void `[`pop`](#classkpsr_1_1mem_1_1SafeQueue_1abd44052b41a0e87237897849efa5a3fc)`(value_type & item)` 

pop Pops item from the queue. If queue is empty, this function blocks until item becomes available.

#### Parameters
* `item` The item.

#### `public inline void `[`move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1aa5197ae1c039640c717fb52d5de4449c)`(value_type & item)` 

move_pop Pops item from the queue using the contained type's move assignment operator, if it has one. This method is identical to the [pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1abd44052b41a0e87237897849efa5a3fc) method if that type has no move assignment operator. If queue is empty, this function blocks until item becomes available.

#### Parameters
* `item` The item.

#### `public inline bool `[`try_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a)`(value_type & item)` 

try_pop Tries to pop item from the queue.

#### Parameters
* `item` The item. 

#### Returns
False is returned if no item is available.

#### `public inline bool `[`try_move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a4bd633b1d7bbf4b370cff51dceacf93e)`(value_type & item)` 

try_move_pop Tries to pop item from the queue using the contained type's move assignment operator, if it has one. This method is identical to the [try_pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a) method if that type has no move assignment operator.

#### Parameters
* `item` The item. 

#### Returns
False is returned if no item is available.

#### `public inline bool `[`timeout_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a2556d0ab30c2cc84ac74cd13e28422d9)`(value_type & item,std::uint64_t timeout)` 

timeout_pop Pops item from the queue. If the queue is empty, blocks for timeout microseconds, or until item becomes available.

#### Parameters
* `item` An item. 

* `timeout` The number of microseconds to wait. 

#### Returns
true if get an item from the queue, false if no item is received before the timeout.

#### `public inline bool `[`timeout_move_pop`](#classkpsr_1_1mem_1_1SafeQueue_1a2ecb978a337d0122ad029723a19ac01f)`(value_type & item,std::uint64_t timeout)` 

timeout_move_pop Pops item from the queue using the contained type's move assignment operator, if it has one.. If the queue is empty, blocks for timeout microseconds, or until item becomes available. This method is identical to the [try_pop()](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue_1a6d9d8ccb8497cd250192074dc943f76a) method if that type has no move assignment operator.

#### Parameters
* `item` An item. 

* `timeout` The number of microseconds to wait. 

#### Returns

#### Returns
true if get an item from the queue, false if no item is received before the timeout.

#### `public inline size_type `[`size`](#classkpsr_1_1mem_1_1SafeQueue_1a2022ee7832d5c01aa6af8a8da7e08dac)`() const` 

size Gets the number of items in the queue.

#### Returns
Number of items in the queue.

#### `public inline bool `[`empty`](#classkpsr_1_1mem_1_1SafeQueue_1a42fc93d83ccb308a3893fb0af5fb593a)`() const` 

empty Check if the queue is empty.

#### Returns
true if queue is empty.

#### `public inline bool `[`full`](#classkpsr_1_1mem_1_1SafeQueue_1ad7e81223f4c2fad7f82e314e3db719a0)`() const` 

full Check if the queue is full.

#### Returns
true if queue is full.

#### `public inline void `[`swap`](#classkpsr_1_1mem_1_1SafeQueue_1a56b5f32fe0543235506794546d42c9d3)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` 

swap Swaps the contents.

#### Parameters
* `sq` The [SafeQueue](#classkpsr_1_1mem_1_1SafeQueue) to swap with 'this'.

#### `public inline `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & `[`operator=`](#classkpsr_1_1mem_1_1SafeQueue_1a3b849d0a69bd1e730219650dff2c8e10)`(const `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & sq)` 

operator = The copy assignment operator

#### Parameters
* `sq` 

#### Returns

#### `public inline `[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` & `[`operator=`](#classkpsr_1_1mem_1_1SafeQueue_1a53cf5c9071a8d82906fb6587a84bdc37)`(`[`SafeQueue`](#classkpsr_1_1mem_1_1SafeQueue)` && sq)` 

operator = The move assignment operator

#### Parameters
* `sq` 

#### Returns

# class `kpsr::mem::SafeQueuePoller` 

```
class kpsr::mem::SafeQueuePoller
  : public kpsr::mem::InMemoryQueuePoller
```  

The [SafeQueuePoller](#classkpsr_1_1mem_1_1SafeQueuePoller) class.

2023 Klepsydra Technologies AG

2.1.0

This class, which extends from the [event_emitter_subscriber.h](#event__emitter__subscriber_8h_source) is an asynchronous in-memory middleware. It block the subcriber thread when no events are available. This class uses the locking [SafeQueue](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue) for the loop.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`SafeQueuePoller`](#classkpsr_1_1mem_1_1SafeQueuePoller_1a319b1b9858f5f5b621aa7a2bb14da2b5)`(`[`SafeQueue](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue)< [EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS)` | [SafeQueuePoller](#classkpsr_1_1mem_1_1SafeQueuePoller).

## Members

#### `public inline  `[`SafeQueuePoller`](#classkpsr_1_1mem_1_1SafeQueuePoller_1a319b1b9858f5f5b621aa7a2bb14da2b5)`(`[`SafeQueue](api-kpsr-mem-composition.md#classkpsr_1_1mem_1_1SafeQueue)< [EventData`](api-kpsr-mem-composition.md#structkpsr_1_1mem_1_1EventData)`< const T >> & safeQueue,`[`SafeEventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName,unsigned int sleepPeriodUS)` 

[SafeQueuePoller](#classkpsr_1_1mem_1_1SafeQueuePoller).

#### Parameters
* `safeQueue` 

* `eventEmitter` 

* `eventName` 

* `sleepPeriodUS`

# struct `kpsr::mem::EventData` 

The [EventData](#structkpsr_1_1mem_1_1EventData) struct.

2023 Klepsydra Technologies AG

2.1.0

wrapper struct to be used when storing events in the queue.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::shared_ptr< T > `[`eventData`](#structkpsr_1_1mem_1_1EventData_1ae89435d6ec615b3bd4603eb280408f94) | eventData actual event
`public long long unsigned int `[`enqueuedTimeInNs`](#structkpsr_1_1mem_1_1EventData_1a7df1d196a1cafa92c1a3b3ab69ef0472) | enqueuedTimeInNs timestamp at which the event was placed in the queue.

## Members

#### `public std::shared_ptr< T > `[`eventData`](#structkpsr_1_1mem_1_1EventData_1ae89435d6ec615b3bd4603eb280408f94) 

eventData actual event

#### `public long long unsigned int `[`enqueuedTimeInNs`](#structkpsr_1_1mem_1_1EventData_1a7df1d196a1cafa92c1a3b3ab69ef0472) 

enqueuedTimeInNs timestamp at which the event was placed in the queue.

