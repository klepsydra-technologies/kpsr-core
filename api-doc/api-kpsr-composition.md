<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-composition` 

This group of classes relates exclusively to the assemblying of the application. In Spring terms, the 'wiring' of the application is done using this API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::EventEmitter`](#classkpsr_1_1EventEmitter) | The [EventEmitter](#classkpsr_1_1EventEmitter) class.
`class `[`kpsr::EventEmitterSubscriber`](#classkpsr_1_1EventEmitterSubscriber) | The [EventEmitterSubscriber](#classkpsr_1_1EventEmitterSubscriber) class.
`class `[`kpsr::FromMiddlewareChannel`](#classkpsr_1_1FromMiddlewareChannel) | The [FromMiddlewareChannel](#classkpsr_1_1FromMiddlewareChannel) class.
`class `[`kpsr::ObjectPoolPublisher`](#classkpsr_1_1ObjectPoolPublisher) | The [ObjectPoolPublisher](#classkpsr_1_1ObjectPoolPublisher) class.
`class `[`kpsr::PropertyFileEnvironment`](#classkpsr_1_1PropertyFileEnvironment) | The [PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment) class.
`class `[`kpsr::ToMiddlewareChannel`](#classkpsr_1_1ToMiddlewareChannel) | The [ToMiddlewareChannel](#classkpsr_1_1ToMiddlewareChannel) class.
`struct `[`kpsr::EventEmitter::ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase) | 
`struct `[`kpsr::EventEmitter::Listener`](#structkpsr_1_1EventEmitter_1_1Listener) | 

# class `kpsr::EventEmitter` 

The [EventEmitter](#classkpsr_1_1EventEmitter) class.

Klepsydra Technologies 2019-2020.

2.1.0

An event emitter pattern implementation oriented to Klepsydra API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::map< unsigned int, std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > > `[`_listenerStats`](#classkpsr_1_1EventEmitter_1ad45be55b007e60948ea25b765d7d7914) | _listenerStats
`public  `[`EventEmitter`](#classkpsr_1_1EventEmitter_1a4b592c9fbff30ab1650e9bd299a881d3)`()` | [EventEmitter](#classkpsr_1_1EventEmitter).
`public  `[`~EventEmitter`](#classkpsr_1_1EventEmitter_1ac5d4165185099b8f8abdb2f765be68e1)`()` | 
`public template<>`  <br/>`unsigned int `[`add_listener`](#classkpsr_1_1EventEmitter_1a01e0517373a71c991ef1e132b72c3b1e)`(std::string event_id,std::string listener_name,bool isOnce,std::function< void(const Args &...)> cb)` | add_listener
`public unsigned int `[`add_listener`](#classkpsr_1_1EventEmitter_1a32b21c1d9f922927095577a711b72325)`(std::string event_id,std::string listener_name,bool isOnce,std::function< void()> cb)` | add_listener
`public template<>`  <br/>`unsigned int `[`on`](#classkpsr_1_1EventEmitter_1a13a9b8d8c7d5eff6407a11b64e550fb3)`(std::string event_id,std::string listener_name,std::function< void(const Args &...)> cb)` | on
`public unsigned int `[`on`](#classkpsr_1_1EventEmitter_1a5ed1f39edab4c47a188ffcc491aab836)`(std::string event_id,std::string listener_name,std::function< void()> cb)` | on
`public template<>`  <br/>`unsigned int `[`once`](#classkpsr_1_1EventEmitter_1a1781b1c9817b6df5f354fbd9e719d4a0)`(std::string event_id,std::function< void(const Args &...)> cb)` | once
`public unsigned int `[`once`](#classkpsr_1_1EventEmitter_1ad31d0863b5750231963cbd7a637a3344)`(std::string event_id,std::function< void()> cb)` | once
`public void `[`remove_listener`](#classkpsr_1_1EventEmitter_1a26385a55296b51e55577791011f4a4f6)`(unsigned int listener_id)` | remove_listener
`public template<>`  <br/>`void `[`emitEvent`](#classkpsr_1_1EventEmitter_1a9c34517b84e5e1a3678d9fb3012c81c9)`(std::string event_id,long long unsigned int enqueuedTimeNs,const Args &... args)` | emitEvent

## Members

#### `public std::map< unsigned int, std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > > `[`_listenerStats`](#classkpsr_1_1EventEmitter_1ad45be55b007e60948ea25b765d7d7914) 

_listenerStats

A map that contains all the subscription stats associated to each listener in the event emitter.

#### `public  `[`EventEmitter`](#classkpsr_1_1EventEmitter_1a4b592c9fbff30ab1650e9bd299a881d3)`()` 

[EventEmitter](#classkpsr_1_1EventEmitter).

#### `public  `[`~EventEmitter`](#classkpsr_1_1EventEmitter_1ac5d4165185099b8f8abdb2f765be68e1)`()` 

#### `public template<>`  <br/>`unsigned int `[`add_listener`](#classkpsr_1_1EventEmitter_1a01e0517373a71c991ef1e132b72c3b1e)`(std::string event_id,std::string listener_name,bool isOnce,std::function< void(const Args &...)> cb)` 

add_listener

#### Parameters
* `event_id` 

* `listener_name` 

* `isOnce` 

* `cb` 

#### Returns

#### `public unsigned int `[`add_listener`](#classkpsr_1_1EventEmitter_1a32b21c1d9f922927095577a711b72325)`(std::string event_id,std::string listener_name,bool isOnce,std::function< void()> cb)` 

add_listener

#### Parameters
* `event_id` 

* `listener_name` 

* `isOnce` 

* `cb` 

#### Returns

#### `public template<>`  <br/>`unsigned int `[`on`](#classkpsr_1_1EventEmitter_1a13a9b8d8c7d5eff6407a11b64e550fb3)`(std::string event_id,std::string listener_name,std::function< void(const Args &...)> cb)` 

on

#### Parameters
* `event_id` 

* `listener_name` 

* `cb` 

#### Returns

#### `public unsigned int `[`on`](#classkpsr_1_1EventEmitter_1a5ed1f39edab4c47a188ffcc491aab836)`(std::string event_id,std::string listener_name,std::function< void()> cb)` 

on

#### Parameters
* `event_id` 

* `listener_name` 

* `cb` 

#### Returns

#### `public template<>`  <br/>`unsigned int `[`once`](#classkpsr_1_1EventEmitter_1a1781b1c9817b6df5f354fbd9e719d4a0)`(std::string event_id,std::function< void(const Args &...)> cb)` 

once

#### Parameters
* `event_id` 

* `cb` 

#### Returns

#### `public unsigned int `[`once`](#classkpsr_1_1EventEmitter_1ad31d0863b5750231963cbd7a637a3344)`(std::string event_id,std::function< void()> cb)` 

once

#### Parameters
* `event_id` 

* `cb` 

#### Returns

#### `public void `[`remove_listener`](#classkpsr_1_1EventEmitter_1a26385a55296b51e55577791011f4a4f6)`(unsigned int listener_id)` 

remove_listener

#### Parameters
* `listener_id`

#### `public template<>`  <br/>`void `[`emitEvent`](#classkpsr_1_1EventEmitter_1a9c34517b84e5e1a3678d9fb3012c81c9)`(std::string event_id,long long unsigned int enqueuedTimeNs,const Args &... args)` 

emitEvent

#### Parameters
* `event_id` 

* `enqueuedTimeNs` 

* `args`

# class `kpsr::EventEmitterSubscriber` 

```
class kpsr::EventEmitterSubscriber
  : public kpsr::Subscriber< T >
```  

The [EventEmitterSubscriber](#classkpsr_1_1EventEmitterSubscriber) class.

Klepsydra Technologies 2019-2020.

2.1.0

Main implementation of the subscriber based on the event emitter class. It is basically a wrapper to the event emitter subscription API It is used in most of the Klepsydra subscription implementations.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`EventEmitterSubscriber`](#classkpsr_1_1EventEmitterSubscriber_1ac0d5f141b4dc9bd9e87326bcfa0323fc)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName)` | [EventEmitterSubscriber](#classkpsr_1_1EventEmitterSubscriber).
`public inline virtual void `[`registerListenerOnce`](#classkpsr_1_1EventEmitterSubscriber_1a11340ddadf4a47f103b5426e79141776)`(const std::function< void(const T &)> listener)` | registerListenerOnce
`public inline virtual void `[`registerListener`](#classkpsr_1_1EventEmitterSubscriber_1a00d4388f2deb604cfaafd5a875f83d1a)`(const std::string name,const std::function< void(const T &)> listener)` | registerListener
`public inline virtual void `[`removeListener`](#classkpsr_1_1EventEmitterSubscriber_1abed78456932b73df34df1f48a518c40f)`(const std::string name)` | removeListener
`public inline virtual std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1EventEmitterSubscriber_1a382834d691595557068e43bab9ce9f01)`(const std::string name)` | getSubscriptionStats retrieves the performance information of the listener.

## Members

#### `public inline  `[`EventEmitterSubscriber`](#classkpsr_1_1EventEmitterSubscriber_1ac0d5f141b4dc9bd9e87326bcfa0323fc)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,`[`EventEmitter`](api-kpsr-composition.md#classkpsr_1_1EventEmitter)` & eventEmitter,std::string eventName)` 

[EventEmitterSubscriber](#classkpsr_1_1EventEmitterSubscriber).

#### Parameters
* `container` If not null, this container will be used to register all listeners passing through this subscriber instance. 

* `eventEmitter` The event emitter instance. 

* `eventName` A name to be assigned to this subscriber.

#### `public inline virtual void `[`registerListenerOnce`](#classkpsr_1_1EventEmitterSubscriber_1a11340ddadf4a47f103b5426e79141776)`(const std::function< void(const T &)> listener)` 

registerListenerOnce

#### Parameters
* `listener`

#### `public inline virtual void `[`registerListener`](#classkpsr_1_1EventEmitterSubscriber_1a00d4388f2deb604cfaafd5a875f83d1a)`(const std::string name,const std::function< void(const T &)> listener)` 

registerListener

#### Parameters
* `name` 

* `listener`

#### `public inline virtual void `[`removeListener`](#classkpsr_1_1EventEmitterSubscriber_1abed78456932b73df34df1f48a518c40f)`(const std::string name)` 

removeListener

#### Parameters
* `name`

#### `public inline virtual std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1EventEmitterSubscriber_1a382834d691595557068e43bab9ce9f01)`(const std::string name)` 

getSubscriptionStats retrieves the performance information of the listener.

#### Parameters
* `name`

# class `kpsr::FromMiddlewareChannel` 

The [FromMiddlewareChannel](#classkpsr_1_1FromMiddlewareChannel) class.

Klepsydra Technologies 2019-2020.

2.1.0

Internal facility abstract class for reading data from middleware. Concrete implementations are available for ZMQ, ROS and DDS.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`FromMiddlewareChannel`](#classkpsr_1_1FromMiddlewareChannel_1a3bbdbfcd57d5a03796c09f43e5fc91bf)`(`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< KpsrClass > * internalPublisher)` | [FromMiddlewareChannel](#classkpsr_1_1FromMiddlewareChannel).
`public inline void `[`onMiddlewareMessage`](#classkpsr_1_1FromMiddlewareChannel_1af53c02272a1b1c094e41e3518f6f9c79)`(const MddlwClass & message)` | onMiddlewareMessage

## Members

#### `public inline  `[`FromMiddlewareChannel`](#classkpsr_1_1FromMiddlewareChannel_1a3bbdbfcd57d5a03796c09f43e5fc91bf)`(`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< KpsrClass > * internalPublisher)` 

[FromMiddlewareChannel](#classkpsr_1_1FromMiddlewareChannel).

#### Parameters
* `internalPublisher`

#### `public inline void `[`onMiddlewareMessage`](#classkpsr_1_1FromMiddlewareChannel_1af53c02272a1b1c094e41e3518f6f9c79)`(const MddlwClass & message)` 

onMiddlewareMessage

#### Parameters
* `message`

# class `kpsr::ObjectPoolPublisher` 

```
class kpsr::ObjectPoolPublisher
  : public kpsr::Publisher< T >
```  

The [ObjectPoolPublisher](#classkpsr_1_1ObjectPoolPublisher) class.

Klepsydra Technologies 2019-2020.

2.1.0

Abstract publisher based for most of the concrete implementations (ROS, DDS, ZMQ and event loop.). It has an object pool that pre allocates objects before copying for publication.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ObjectPoolPublisher`](#classkpsr_1_1ObjectPoolPublisher_1abb2dfb9eb12503de440fcfa7dfdc8d46)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,const std::string type,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` | [ObjectPoolPublisher](#classkpsr_1_1ObjectPoolPublisher).
`public inline virtual void `[`processAndPublish`](#classkpsr_1_1ObjectPoolPublisher_1a04be79329a480929b14ba857676b2ecc)`(std::function< void(T &)> process)` | processAndPublish

## Members

#### `public inline  `[`ObjectPoolPublisher`](#classkpsr_1_1ObjectPoolPublisher_1abb2dfb9eb12503de440fcfa7dfdc8d46)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,const std::string name,const std::string type,int poolSize,std::function< void(T &)> initializerFunction,std::function< void(const T &, T &)> eventCloner)` 

[ObjectPoolPublisher](#classkpsr_1_1ObjectPoolPublisher).

#### Parameters
* `container` [Container](api-kpsr-monitoring.md#classkpsr_1_1Container) to attach in case of monitoring. 

* `name` name of the publisher 

* `type` valid values are EVENT_EMITTER, ROS, etc. 

* `poolSize` size of the pool to create. 

* `initializerFunction` init function to execute after creation. 

* `eventCloner` optional function to use instead of the copy.

#### `public inline virtual void `[`processAndPublish`](#classkpsr_1_1ObjectPoolPublisher_1a04be79329a480929b14ba857676b2ecc)`(std::function< void(T &)> process)` 

processAndPublish

#### Parameters
* `process`

# class `kpsr::PropertyFileEnvironment` 

```
class kpsr::PropertyFileEnvironment
  : public kpsr::Environment
```  

The [PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment) class.

Klepsydra Technologies 2019-2020.

2.1.0

A PROP_FILE based implementation of the environment. Properties are read and written to the member PROP_FILE file.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`PropertyFileEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a0828ca822ebe3b9427070932f87dd61c)`(const std::string propertyFileName)` | [PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment).
`public  `[`PropertyFileEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a92c95467fbf80fba72ab53ef137b1835)`(std::istream & propertyFileContent)` | [PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment).
`public virtual void `[`getPropertyString`](#classkpsr_1_1PropertyFileEnvironment_1a6bd2ce6acb130d0a6b9a188ec57e6c03)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1PropertyFileEnvironment_1a2663c0a4d59bba95cba6a8e2ec5e643d)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1PropertyFileEnvironment_1ad5b0aeedb9efb417c8015c9d2c0270e8)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1PropertyFileEnvironment_1ab23f97859a53166f0fd1de5a62130140)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1PropertyFileEnvironment_1a822446138f4e09aa793e5e48442d6d30)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1PropertyFileEnvironment_1a382567f585d3ebc73fcaa184799c9a61)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1PropertyFileEnvironment_1a47ea301bd3d53e1a0e03fc5683841ba1)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1PropertyFileEnvironment_1a3433c0c84db32b4d361df331316b52eb)`(const std::string key,const bool & value)` | setPropertyBool
`public virtual void `[`persist`](#classkpsr_1_1PropertyFileEnvironment_1a7099e99fc8bf315255f9fc599bb590a9)`()` | persist
`public void `[`reload`](#classkpsr_1_1PropertyFileEnvironment_1af6c21be9f7d0e74fbc28ed77566b92a0)`(std::string propertyFileContent)` | reload
`public std::string `[`exportEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a737b3adb46246ac4b24d0fdd34b22775)`()` | exportEnvironment
`protected `[`ConfigFile`](api-undefined.md#classkpsr_1_1ConfigFile)` `[`_configFile`](#classkpsr_1_1PropertyFileEnvironment_1a1d491d7417d5e2bbc04814605f12c049) | 

## Members

#### `public  `[`PropertyFileEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a0828ca822ebe3b9427070932f87dd61c)`(const std::string propertyFileName)` 

[PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment).

#### Parameters
* `propertyFileFileName`

#### `public  `[`PropertyFileEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a92c95467fbf80fba72ab53ef137b1835)`(std::istream & propertyFileContent)` 

[PropertyFileEnvironment](#classkpsr_1_1PropertyFileEnvironment).

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1PropertyFileEnvironment_1a6bd2ce6acb130d0a6b9a188ec57e6c03)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1PropertyFileEnvironment_1a2663c0a4d59bba95cba6a8e2ec5e643d)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1PropertyFileEnvironment_1ad5b0aeedb9efb417c8015c9d2c0270e8)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1PropertyFileEnvironment_1ab23f97859a53166f0fd1de5a62130140)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1PropertyFileEnvironment_1a822446138f4e09aa793e5e48442d6d30)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1PropertyFileEnvironment_1a382567f585d3ebc73fcaa184799c9a61)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1PropertyFileEnvironment_1a47ea301bd3d53e1a0e03fc5683841ba1)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1PropertyFileEnvironment_1a3433c0c84db32b4d361df331316b52eb)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`persist`](#classkpsr_1_1PropertyFileEnvironment_1a7099e99fc8bf315255f9fc599bb590a9)`()` 

persist

#### `public void `[`reload`](#classkpsr_1_1PropertyFileEnvironment_1af6c21be9f7d0e74fbc28ed77566b92a0)`(std::string propertyFileContent)` 

reload

#### Parameters
* `propertyFileContent`

#### `public std::string `[`exportEnvironment`](#classkpsr_1_1PropertyFileEnvironment_1a737b3adb46246ac4b24d0fdd34b22775)`()` 

exportEnvironment

#### Returns

#### `protected `[`ConfigFile`](api-undefined.md#classkpsr_1_1ConfigFile)` `[`_configFile`](#classkpsr_1_1PropertyFileEnvironment_1a1d491d7417d5e2bbc04814605f12c049) 

# class `kpsr::ToMiddlewareChannel` 

```
class kpsr::ToMiddlewareChannel
  : public kpsr::Publisher< KpsrClass >
```  

The [ToMiddlewareChannel](#classkpsr_1_1ToMiddlewareChannel) class.

Klepsydra Technologies 2019-2020.

2.1.0

Abstract class to transform and publish to the middleware.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ToMiddlewareChannel`](#classkpsr_1_1ToMiddlewareChannel_1aa41a305f1c17b05ebe6c0dc1ee3dc51f)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< MddlwClass > * middlewarePublisher)` | [ToMiddlewareChannel](#classkpsr_1_1ToMiddlewareChannel).
`public inline virtual void `[`processAndPublish`](#classkpsr_1_1ToMiddlewareChannel_1ae28e004b0e34dc8a95eef5c312857000)`(std::function< void(KpsrClass &)> process)` | processAndPublish

## Members

#### `public inline  `[`ToMiddlewareChannel`](#classkpsr_1_1ToMiddlewareChannel_1aa41a305f1c17b05ebe6c0dc1ee3dc51f)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< MddlwClass > * middlewarePublisher)` 

[ToMiddlewareChannel](#classkpsr_1_1ToMiddlewareChannel).

#### Parameters
* `container` 

* `middlewarePublisher` real native middleware publisher

#### `public inline virtual void `[`processAndPublish`](#classkpsr_1_1ToMiddlewareChannel_1ae28e004b0e34dc8a95eef5c312857000)`(std::function< void(KpsrClass &)> process)` 

processAndPublish

#### Parameters
* `process`

# struct `kpsr::EventEmitter::ListenerBase` 

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public unsigned int `[`id`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a75ece21029cf85db43da27bf5409946a) | 
`public bool `[`isOnce`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a55c49827eee5f8e148b05d81da2f759f) | 
`public inline  `[`ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a988a377372dfc933bff0087a273ed2de)`()` | 
`public inline  `[`ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a7db265b072598152c54f9062d3fa0f9f)`(unsigned int i,bool isOnce)` | 
`public inline virtual  `[`~ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a209fb463e3ccc305239a3eb03267bd6f)`()` | 

## Members

#### `public unsigned int `[`id`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a75ece21029cf85db43da27bf5409946a) 

#### `public bool `[`isOnce`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a55c49827eee5f8e148b05d81da2f759f) 

#### `public inline  `[`ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a988a377372dfc933bff0087a273ed2de)`()` 

#### `public inline  `[`ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a7db265b072598152c54f9062d3fa0f9f)`(unsigned int i,bool isOnce)` 

#### `public inline virtual  `[`~ListenerBase`](#structkpsr_1_1EventEmitter_1_1ListenerBase_1a209fb463e3ccc305239a3eb03267bd6f)`()` 

# struct `kpsr::EventEmitter::Listener` 

```
struct kpsr::EventEmitter::Listener
  : public kpsr::EventEmitter::ListenerBase
```  

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::function< void(const Args &...)> `[`cb`](#structkpsr_1_1EventEmitter_1_1Listener_1ae706314658beac1b3762fdd6e433cc4c) | 
`public inline  `[`Listener`](#structkpsr_1_1EventEmitter_1_1Listener_1ab7456a83c7ab308c313debc88733ccfb)`()` | 
`public inline  `[`Listener`](#structkpsr_1_1EventEmitter_1_1Listener_1a9cb15b63df69712b31affc44f089cb2d)`(unsigned int i,bool isOnce,std::function< void(const Args &...)> c)` | 

## Members

#### `public std::function< void(const Args &...)> `[`cb`](#structkpsr_1_1EventEmitter_1_1Listener_1ae706314658beac1b3762fdd6e433cc4c) 

#### `public inline  `[`Listener`](#structkpsr_1_1EventEmitter_1_1Listener_1ab7456a83c7ab308c313debc88733ccfb)`()` 

#### `public inline  `[`Listener`](#structkpsr_1_1EventEmitter_1_1Listener_1a9cb15b63df69712b31affc44f089cb2d)`(unsigned int i,bool isOnce,std::function< void(const Args &...)> c)` 

