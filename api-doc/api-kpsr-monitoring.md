<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-monitoring` 

This group of classes is intended to monitor the performance of the application along with allowing some administration tasks like restart services and read and write configurration.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::Container`](#classkpsr_1_1Container) | The [Container](#classkpsr_1_1Container) class.
`class `[`kpsr::TimeUtils`](#classkpsr_1_1TimeUtils) | The [TimeUtils](#classkpsr_1_1TimeUtils) class.
`struct `[`kpsr::FunctionStats`](#structkpsr_1_1FunctionStats) | [FunctionStats](#structkpsr_1_1FunctionStats) class.
`struct `[`kpsr::PublicationStats`](#structkpsr_1_1PublicationStats) | [PublicationStats](#structkpsr_1_1PublicationStats) class.
`struct `[`kpsr::ServiceStats`](#structkpsr_1_1ServiceStats) | [ServiceStats](#structkpsr_1_1ServiceStats) class.
`struct `[`kpsr::SubscriptionStats`](#structkpsr_1_1SubscriptionStats) | [SubscriptionStats](#structkpsr_1_1SubscriptionStats) class.

# class `kpsr::Container` 

The [Container](#classkpsr_1_1Container) class.

Klepsydra Technologies 2019-2020.

2.1.0

This class keeps a reference to the statistics object in each service. subscriber and publisher in the system. Custom methods statistics can also be registered here.

Services, publishers and subscriber will register themselves when they receive a non-null pointer to a container in their constructor.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`Container`](#classkpsr_1_1Container_1a03abe363bfc66f31298d3fc7911d53e6)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * env,std::string applicationName)` | [Container](#classkpsr_1_1Container) constructor.
`public virtual void `[`start`](#classkpsr_1_1Container_1a41321600928e5d01ba867361c6b3b4f6)`()` | start all master services.
`public virtual void `[`stop`](#classkpsr_1_1Container_1a6f8037445aa3c309086421ea5583828e)`()` | stop all services.
`public void `[`attach`](#classkpsr_1_1Container_1ad81916db5ebbebfa1919b2841d945a52)`(`[`Service`](api-kpsr-application.md#classkpsr_1_1Service)` * service)` | attach a service and its statistics variable to the container
`public void `[`attach`](#classkpsr_1_1Container_1aa9b1eec44287300edaf210b6b69c2cd6)`(`[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * functionStats)` | attach a custom method statistics to the container
`public void `[`detach`](#classkpsr_1_1Container_1a0914b7fc9dd620b35be9e8963197abda)`(`[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * functionStats)` | detach
`public void `[`attach`](#classkpsr_1_1Container_1ac221a18efa742ee7db647f8f771a902c)`(`[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` * serviceStats)` | attach a service statistics to the container
`public void `[`attach`](#classkpsr_1_1Container_1afb727bf262a28a419beeb447ea03b456)`(`[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` * publicationStats)` | attach a publication statistics to the container
`public void `[`attach`](#classkpsr_1_1Container_1a39ccb13710e32a16d620d4d0afff395f)`(`[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * subscriptionStats)` | attach a subscription statistics to the container
`public void `[`detach`](#classkpsr_1_1Container_1a615183e9b178052b2f23d4d7d32584a8)`(`[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * subscriptionStats)` | detach
`protected `[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * `[`_env`](#classkpsr_1_1Container_1a87701d36190530768eb0e08a950e0460) | 
`protected std::string `[`_applicationName`](#classkpsr_1_1Container_1a43fbcb62b5a641b668ac8493dbaad1b8) | 
`protected std::atomic_bool `[`_running`](#classkpsr_1_1Container_1acc88b39cd28ce7566f4f99a8f4b17840) | 
`protected std::vector< `[`Service`](api-kpsr-application.md#classkpsr_1_1Service)` * > `[`_managedServices`](#classkpsr_1_1Container_1a0c1aa8eaa512be9b300371ae4ba43a7f) | 
`protected std::vector< `[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * > `[`_functionStats`](#classkpsr_1_1Container_1a48826f562153972203f0337cd8c238db) | 
`protected std::vector< `[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` * > `[`_serviceStats`](#classkpsr_1_1Container_1a956b8535eb2d085183f1ef9c0e2f0510) | 
`protected std::vector< `[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` * > `[`_publicationStats`](#classkpsr_1_1Container_1a7cb015a40191f4a631686d3364d2d525) | 
`protected std::vector< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * > `[`_subscriptionStats`](#classkpsr_1_1Container_1a205e05962070b049f9b6058819b432ee) | 
`protected mutable std::mutex `[`_serviceMutex`](#classkpsr_1_1Container_1a334a0f9276c87134525479508e60f6f2) | 
`protected mutable std::mutex `[`_serviceStatsMutex`](#classkpsr_1_1Container_1a7450ac6bbb6d2033373283f966305a65) | 
`protected mutable std::mutex `[`_publishStatsMutex`](#classkpsr_1_1Container_1aa3bd4f65a528f53307f44ec60732c1a3) | 
`protected mutable std::mutex `[`_subscriptionStatsMutex`](#classkpsr_1_1Container_1aea581854fa9653f6ea1e569ea61544d3) | 
`protected mutable std::mutex `[`_functionStatsMutex`](#classkpsr_1_1Container_1a5d11fc14b848c9e1c1e48be903809121) | 

## Members

#### `public  `[`Container`](#classkpsr_1_1Container_1a03abe363bfc66f31298d3fc7911d53e6)`(`[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * env,std::string applicationName)` 

[Container](#classkpsr_1_1Container) constructor.

#### Parameters
* `env` to administer 

* `applicationName` to use as id of the process running these services.

#### `public virtual void `[`start`](#classkpsr_1_1Container_1a41321600928e5d01ba867361c6b3b4f6)`()` 

start all master services.

#### `public virtual void `[`stop`](#classkpsr_1_1Container_1a6f8037445aa3c309086421ea5583828e)`()` 

stop all services.

#### `public void `[`attach`](#classkpsr_1_1Container_1ad81916db5ebbebfa1919b2841d945a52)`(`[`Service`](api-kpsr-application.md#classkpsr_1_1Service)` * service)` 

attach a service and its statistics variable to the container

#### Parameters
* `service` Pointer to a service

#### `public void `[`attach`](#classkpsr_1_1Container_1aa9b1eec44287300edaf210b6b69c2cd6)`(`[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * functionStats)` 

attach a custom method statistics to the container

#### Parameters
* `functionStats` Pointer to a basic stat object.

#### `public void `[`detach`](#classkpsr_1_1Container_1a0914b7fc9dd620b35be9e8963197abda)`(`[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * functionStats)` 

detach

#### Parameters
* `functionStats`

#### `public void `[`attach`](#classkpsr_1_1Container_1ac221a18efa742ee7db647f8f771a902c)`(`[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` * serviceStats)` 

attach a service statistics to the container

#### Parameters
* `serviceStats` Pointer to a service stats.

#### `public void `[`attach`](#classkpsr_1_1Container_1afb727bf262a28a419beeb447ea03b456)`(`[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` * publicationStats)` 

attach a publication statistics to the container

#### Parameters
* `publicationStats` Pointer to a publication stats.

#### `public void `[`attach`](#classkpsr_1_1Container_1a39ccb13710e32a16d620d4d0afff395f)`(`[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * subscriptionStats)` 

attach a subscription statistics to the container

#### Parameters
* `subscriptionStats` Pointer to a subscription stats.

#### `public void `[`detach`](#classkpsr_1_1Container_1a615183e9b178052b2f23d4d7d32584a8)`(`[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * subscriptionStats)` 

detach

#### Parameters
* `subscriptionStats`

#### `protected `[`Environment`](api-kpsr-application.md#classkpsr_1_1Environment)` * `[`_env`](#classkpsr_1_1Container_1a87701d36190530768eb0e08a950e0460) 

#### `protected std::string `[`_applicationName`](#classkpsr_1_1Container_1a43fbcb62b5a641b668ac8493dbaad1b8) 

#### `protected std::atomic_bool `[`_running`](#classkpsr_1_1Container_1acc88b39cd28ce7566f4f99a8f4b17840) 

#### `protected std::vector< `[`Service`](api-kpsr-application.md#classkpsr_1_1Service)` * > `[`_managedServices`](#classkpsr_1_1Container_1a0c1aa8eaa512be9b300371ae4ba43a7f) 

#### `protected std::vector< `[`FunctionStats`](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats)` * > `[`_functionStats`](#classkpsr_1_1Container_1a48826f562153972203f0337cd8c238db) 

#### `protected std::vector< `[`ServiceStats`](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats)` * > `[`_serviceStats`](#classkpsr_1_1Container_1a956b8535eb2d085183f1ef9c0e2f0510) 

#### `protected std::vector< `[`PublicationStats`](api-kpsr-monitoring.md#structkpsr_1_1PublicationStats)` * > `[`_publicationStats`](#classkpsr_1_1Container_1a7cb015a40191f4a631686d3364d2d525) 

#### `protected std::vector< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` * > `[`_subscriptionStats`](#classkpsr_1_1Container_1a205e05962070b049f9b6058819b432ee) 

#### `protected mutable std::mutex `[`_serviceMutex`](#classkpsr_1_1Container_1a334a0f9276c87134525479508e60f6f2) 

#### `protected mutable std::mutex `[`_serviceStatsMutex`](#classkpsr_1_1Container_1a7450ac6bbb6d2033373283f966305a65) 

#### `protected mutable std::mutex `[`_publishStatsMutex`](#classkpsr_1_1Container_1aa3bd4f65a528f53307f44ec60732c1a3) 

#### `protected mutable std::mutex `[`_subscriptionStatsMutex`](#classkpsr_1_1Container_1aea581854fa9653f6ea1e569ea61544d3) 

#### `protected mutable std::mutex `[`_functionStatsMutex`](#classkpsr_1_1Container_1a5d11fc14b848c9e1c1e48be903809121) 

# class `kpsr::TimeUtils` 

The [TimeUtils](#classkpsr_1_1TimeUtils) class.

Klepsydra Technologies 2019-2020.

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------

## Members

# struct `kpsr::FunctionStats` 

[FunctionStats](#structkpsr_1_1FunctionStats) class.

Klepsydra Technologies 2019-2020.

2.1.0

Statistics associated to the performance of a customer function. It gathers three main messures: number of invocations, total invocation time, starting time. The use of this class is very simple can be split into three parts: construct, attach to container and surround the code block to be messured. This can be seen in the following example: 
```cpp
class MessuredClass {
public:
   MessuredClass(Container * container)
      : functionStats("messured_class")
   {
      if (container) {
         container->attach(&functionStats);
      }
   }

   void meesuredFunction() {
      functionStats.startProcessMeassure();
      // DO STUFF
      functionStats.stopProcessMeassure();
   }
private:
   FunctionStats functionStats;
}
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public const std::string `[`_name`](#structkpsr_1_1FunctionStats_1a4ac28563e324dcf0ffc9cf1df4d6ffd9) | _name
`public long long unsigned int `[`_totalProcessed`](#structkpsr_1_1FunctionStats_1ab4b1b346957e5e3d21d6bb479818fa0c) | _totalProcessed
`public long long unsigned int `[`_totalProcessingTimeInNanoSecs`](#structkpsr_1_1FunctionStats_1a1fbd309305458e9762173aada3e0165c) | _totalProcessingTimeInNanoSecs
`public inline  `[`FunctionStats`](#structkpsr_1_1FunctionStats_1ac77f3a3cbc427e543712ce9b928864c7)`(const std::string name)` | 
`public inline long long unsigned int `[`getMillisecondsSinceCreation`](#structkpsr_1_1FunctionStats_1a6f6411ccd39c71d0b8e3e7bb17e8953e)`()` | getMillisecondsSinceCreation
`public inline long long unsigned int `[`getMillisecondsSinceStart`](#structkpsr_1_1FunctionStats_1ab9dc8401820c7bd436f8417fb9b0ace6)`()` | getMillisecondsSinceStart
`public inline void `[`startProcessMeassure`](#structkpsr_1_1FunctionStats_1a4fd825acdf0d4bc96ccd5e76127debaf)`()` | 
`public inline void `[`stopProcessMeassure`](#structkpsr_1_1FunctionStats_1a0b27ffacf071cbefd7a6327f4d7d500a)`()` | 
`protected const long long unsigned int `[`_creationTimeMs`](#structkpsr_1_1FunctionStats_1ae91752108f9808ac0e6dc5a45820e741) | 
`protected std::atomic_ullong `[`_beforeTimeNs`](#structkpsr_1_1FunctionStats_1aa0f49ffe743d259cf9de4dd7e339a730) | 
`protected std::atomic_bool `[`_processingStarted`](#structkpsr_1_1FunctionStats_1a7336126c892c122db7b3db2eba6933ea) | 
`protected std::atomic_ullong `[`_processingStartedTimeMs`](#structkpsr_1_1FunctionStats_1ad8e73f70a25d057989e004a877f57899) | 

## Members

#### `public const std::string `[`_name`](#structkpsr_1_1FunctionStats_1a4ac28563e324dcf0ffc9cf1df4d6ffd9) 

_name

#### `public long long unsigned int `[`_totalProcessed`](#structkpsr_1_1FunctionStats_1ab4b1b346957e5e3d21d6bb479818fa0c) 

_totalProcessed

#### `public long long unsigned int `[`_totalProcessingTimeInNanoSecs`](#structkpsr_1_1FunctionStats_1a1fbd309305458e9762173aada3e0165c) 

_totalProcessingTimeInNanoSecs

#### `public inline  `[`FunctionStats`](#structkpsr_1_1FunctionStats_1ac77f3a3cbc427e543712ce9b928864c7)`(const std::string name)` 

#### `public inline long long unsigned int `[`getMillisecondsSinceCreation`](#structkpsr_1_1FunctionStats_1a6f6411ccd39c71d0b8e3e7bb17e8953e)`()` 

getMillisecondsSinceCreation

#### Returns

#### `public inline long long unsigned int `[`getMillisecondsSinceStart`](#structkpsr_1_1FunctionStats_1ab9dc8401820c7bd436f8417fb9b0ace6)`()` 

getMillisecondsSinceStart

#### Returns

#### `public inline void `[`startProcessMeassure`](#structkpsr_1_1FunctionStats_1a4fd825acdf0d4bc96ccd5e76127debaf)`()` 

#### `public inline void `[`stopProcessMeassure`](#structkpsr_1_1FunctionStats_1a0b27ffacf071cbefd7a6327f4d7d500a)`()` 

#### `protected const long long unsigned int `[`_creationTimeMs`](#structkpsr_1_1FunctionStats_1ae91752108f9808ac0e6dc5a45820e741) 

#### `protected std::atomic_ullong `[`_beforeTimeNs`](#structkpsr_1_1FunctionStats_1aa0f49ffe743d259cf9de4dd7e339a730) 

#### `protected std::atomic_bool `[`_processingStarted`](#structkpsr_1_1FunctionStats_1a7336126c892c122db7b3db2eba6933ea) 

#### `protected std::atomic_ullong `[`_processingStartedTimeMs`](#structkpsr_1_1FunctionStats_1ad8e73f70a25d057989e004a877f57899) 

# struct `kpsr::PublicationStats` 

```
struct kpsr::PublicationStats
  : public kpsr::FunctionStats
```  

[PublicationStats](#structkpsr_1_1PublicationStats) class.

Klepsydra Technologies 2019-2020.

2.1.0

Statistics associated to the performance of the publisher. The messures include the [FunctionStats](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats) messures plus discarded event and total event object allocations.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public const std::string `[`_type`](#structkpsr_1_1PublicationStats_1ab5e593a1d0d7bcf36ef2590b7e927050) | _type
`public std::atomic_ullong `[`_totalEventAllocations`](#structkpsr_1_1PublicationStats_1a6f9bbd062b07128ee1f8bc62ca66dc6b) | _totalEventAllocations
`public std::atomic_ullong `[`_totalDiscardedEvents`](#structkpsr_1_1PublicationStats_1a1bafc7c2e170de44adf480cdb12f1861) | _totalDiscardedEvents
`public inline  `[`PublicationStats`](#structkpsr_1_1PublicationStats_1a7b6a873cd5c632dda8fcb710fc10536c)`(const std::string name,const std::string type)` | [ServiceStats](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats).

## Members

#### `public const std::string `[`_type`](#structkpsr_1_1PublicationStats_1ab5e593a1d0d7bcf36ef2590b7e927050) 

_type

#### `public std::atomic_ullong `[`_totalEventAllocations`](#structkpsr_1_1PublicationStats_1a6f9bbd062b07128ee1f8bc62ca66dc6b) 

_totalEventAllocations

#### `public std::atomic_ullong `[`_totalDiscardedEvents`](#structkpsr_1_1PublicationStats_1a1bafc7c2e170de44adf480cdb12f1861) 

_totalDiscardedEvents

#### `public inline  `[`PublicationStats`](#structkpsr_1_1PublicationStats_1a7b6a873cd5c632dda8fcb710fc10536c)`(const std::string name,const std::string type)` 

[ServiceStats](api-kpsr-monitoring.md#structkpsr_1_1ServiceStats).

#### Parameters
* `name` service name to gather stats for. 

* `type` service name to gather stats for.

# struct `kpsr::ServiceStats` 

```
struct kpsr::ServiceStats
  : public kpsr::FunctionStats
```  

[ServiceStats](#structkpsr_1_1ServiceStats) class.

Klepsydra Technologies 2019-2020.

2.1.0

Statistics associated to the performance of the services. The messures include the [FunctionStats](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats) messures plus the total running time. A service can be dynamically stopeed and started during the execution life time of the process.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ServiceStats`](#structkpsr_1_1ServiceStats_1a8df65b653246476cc6e0322f7806d9b2)`(std::string serviceName)` | [ServiceStats](#structkpsr_1_1ServiceStats).
`public inline void `[`startTimeWatch`](#structkpsr_1_1ServiceStats_1a2c5f36108148969efd4377582eadf96e)`()` | startTimeWatch
`public inline void `[`stopTimeWatch`](#structkpsr_1_1ServiceStats_1a2573ac4f94b5310361defcd0ea5ae0a6)`()` | stopTimeWatch
`public inline long long unsigned int `[`getTotalRunningTimeMs`](#structkpsr_1_1ServiceStats_1afe6a74a32b0b4feee4854b493e49f248)`()` | getTotalRunningTimeMs

## Members

#### `public inline  `[`ServiceStats`](#structkpsr_1_1ServiceStats_1a8df65b653246476cc6e0322f7806d9b2)`(std::string serviceName)` 

[ServiceStats](#structkpsr_1_1ServiceStats).

#### Parameters
* `serviceName` service name to gather stats for.

#### `public inline void `[`startTimeWatch`](#structkpsr_1_1ServiceStats_1a2c5f36108148969efd4377582eadf96e)`()` 

startTimeWatch

#### `public inline void `[`stopTimeWatch`](#structkpsr_1_1ServiceStats_1a2573ac4f94b5310361defcd0ea5ae0a6)`()` 

stopTimeWatch

#### `public inline long long unsigned int `[`getTotalRunningTimeMs`](#structkpsr_1_1ServiceStats_1afe6a74a32b0b4feee4854b493e49f248)`()` 

getTotalRunningTimeMs

#### Returns

# struct `kpsr::SubscriptionStats` 

```
struct kpsr::SubscriptionStats
  : public kpsr::FunctionStats
```  

[SubscriptionStats](#structkpsr_1_1SubscriptionStats) class.

Klepsydra Technologies 2019-2020.

2.1.0

Statistics associated to the performance of the listeners. The messures include the [FunctionStats](api-kpsr-monitoring.md#structkpsr_1_1FunctionStats) messures plus the total enqueued time of the events and the number of discarded events. Events can be discarded due to a number of reasons, depending on the actual underlying implementation of the middleware.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public const std::string `[`_type`](#structkpsr_1_1SubscriptionStats_1abf708c0e8ed499747d582b1a42f40c0c) | _subscriberName
`public std::atomic_ullong `[`_totalEnqueuedTimeInNs`](#structkpsr_1_1SubscriptionStats_1af4dc244f8078f49eafb0b84d63d87bb8) | _totalDiscardedEvents
`public std::atomic_ullong `[`_totalDiscardedEvents`](#structkpsr_1_1SubscriptionStats_1a0b316503f0e39caa4daaf7441cc45380) | _totalDiscardedEvents
`public inline  `[`SubscriptionStats`](#structkpsr_1_1SubscriptionStats_1a4fe95731379f2f1fa41c903464c5f839)`(const std::string listenerName,const std::string subscriberName,const std::string type)` | _subscriberName

## Members

#### `public const std::string `[`_type`](#structkpsr_1_1SubscriptionStats_1abf708c0e8ed499747d582b1a42f40c0c) 

_subscriberName

#### `public std::atomic_ullong `[`_totalEnqueuedTimeInNs`](#structkpsr_1_1SubscriptionStats_1af4dc244f8078f49eafb0b84d63d87bb8) 

_totalDiscardedEvents

#### `public std::atomic_ullong `[`_totalDiscardedEvents`](#structkpsr_1_1SubscriptionStats_1a0b316503f0e39caa4daaf7441cc45380) 

_totalDiscardedEvents

#### `public inline  `[`SubscriptionStats`](#structkpsr_1_1SubscriptionStats_1a4fe95731379f2f1fa41c903464c5f839)`(const std::string listenerName,const std::string subscriberName,const std::string type)` 

_subscriberName

#### Parameters
* `listenerName` name of the listener 

* `subscriberName` name of the subscriber containing the listener 

* `type` type of the subscriber for information purpuses (examples are: EVENT_EMITTER, EVENT_LOOP, DISRUPTOR, ROS, ZMQ, DDS)

