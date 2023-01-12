<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-high_performance-composition` 

This group of classes contains the API for assemblying the application using the high_performance. In Spring terms, the 'wiring' of the application is done using this API.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::high_performance::DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider) | The [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider) class.
`class `[`kpsr::high_performance::DataMultiplexerPublisher`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher) | The [DataMultiplexerPublisher](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher) class.
`class `[`kpsr::high_performance::DataMultiplexerSubscriber`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber) | The [DataMultiplexerSubscriber](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber) class.

# class `kpsr::high_performance::DataMultiplexerMiddlewareProvider` 

The [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.0.1

Klepsydra provided wizar for the creation of high_performance and associated pub/sub pairs. Its use is very straight forward as presented in this example: 
```cpp
// Create the high_performance provider
kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4> provider(nullptr, "test");

kpsr::mem::CacheListener<DataMultiplexerTestEvent> eventListener(2);
// get subscriber
provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

for (int i = 0; i < 500; i ++) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    DataMultiplexerTestEvent event(i, "hola");
    // get publisher
    provider.getPublisher()->publish(event);
}
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a4dedef36dfe6422db1ab81b1e27c1c6b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name)` | [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider) basic constructor without any performance tuning params.
`public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a153e42947740a37c9a53373abe95fbdd)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(TEvent &)> eventInitializer)` | [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).
`public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a813e17a0d7b1958c101fa545d77d73a9)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,const TEvent & event)` | [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).
`public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a0c5f0d3984d9e88a1eb5f354f629c287)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(TEvent &)> eventInitializer,std::function< void(const TEvent &, TEvent &)> eventCloner)` | [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).
`public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a263d73343c64a0f8b18651a3f222ad07)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,const TEvent & event,std::function< void(const TEvent &, TEvent &)> eventCloner)` | [DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).
`public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< TEvent > * `[`getPublisher`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1adc37acc1c25c07a4d2b6e376883b3ce0)`()` | getPublisher
`public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< TEvent > * `[`getSubscriber`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a27cac09a956d94cc8f70ecebdf2c1b52)`()` | getSubscriber
`public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< SourceEvent, TEvent > > `[`getProcessForwarder`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1ac8e3b3530eb14ae754fb1684229b9d1b)`(const std::function< void(const SourceEvent &, TEvent &)> & transformFunction)` | getProcessForwarder
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a8502f3b46d209a8976efb8e547a195d3) | 

## Members

#### `public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a4dedef36dfe6422db1ab81b1e27c1c6b)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name)` 

[DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider) basic constructor without any performance tuning params.

#### Parameters
* `container` 

* `name`

#### `public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a153e42947740a37c9a53373abe95fbdd)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(TEvent &)> eventInitializer)` 

[DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).

#### Parameters
* `container` 

* `name` 

* `eventInitializer` function to initialize the events allocated in the ring buffer

#### `public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a813e17a0d7b1958c101fa545d77d73a9)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,const TEvent & event)` 

[DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).

#### Parameters
* `container` 

* `name` 

* `event` used as event to clone the events allocated in the ring buffer.

#### `public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a0c5f0d3984d9e88a1eb5f354f629c287)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(TEvent &)> eventInitializer,std::function< void(const TEvent &, TEvent &)> eventCloner)` 

[DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).

#### Parameters
* `container` 

* `name` 

* `eventInitializer` function to initialize the events allocated in the ring buffer 

* `eventCloner` cloner function used to add new events in the ring buffer when publishing.

#### `public inline  `[`DataMultiplexerMiddlewareProvider`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a263d73343c64a0f8b18651a3f222ad07)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,const TEvent & event,std::function< void(const TEvent &, TEvent &)> eventCloner)` 

[DataMultiplexerMiddlewareProvider](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider).

#### Parameters
* `container` 

* `name` 

* `event` used as event to clone the events allocated in the ring buffer. 

* `eventCloner` cloner function used to add new events in the ring buffer when publishing.

#### `public inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< TEvent > * `[`getPublisher`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1adc37acc1c25c07a4d2b6e376883b3ce0)`()` 

getPublisher

#### Returns

#### `public inline `[`Subscriber`](api-kpsr-application.md#classkpsr_1_1Subscriber)`< TEvent > * `[`getSubscriber`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a27cac09a956d94cc8f70ecebdf2c1b52)`()` 

getSubscriber

#### Returns

#### `public template<>`  <br/>`inline std::shared_ptr< `[`EventTransformForwarder`](api-kpsr-application.md#classkpsr_1_1EventTransformForwarder)`< SourceEvent, TEvent > > `[`getProcessForwarder`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1ac8e3b3530eb14ae754fb1684229b9d1b)`(const std::function< void(const SourceEvent &, TEvent &)> & transformFunction)` 

getProcessForwarder

#### Parameters
* `transformFunction`

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerMiddlewareProvider_1a8502f3b46d209a8976efb8e547a195d3) 

# class `kpsr::high_performance::DataMultiplexerPublisher` 

```
class kpsr::high_performance::DataMultiplexerPublisher
  : public kpsr::Publisher< TEvent >
```  

The [DataMultiplexerPublisher](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher) class.

2023 Klepsydra Technologies AG

2.0.1

This class is not actually used by the client code, but it is documented due to its close relation to the provider. When the method publish is invoked, this class puts a copy of the event into the ringbuffer.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`DataMultiplexerPublisher`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a8d64c3a740187563f51e57ac31144868)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(const TEvent &, TEvent &)> eventCloner,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` | [DataMultiplexerPublisher](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher).
`public inline virtual void `[`processAndPublish`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a1d85e950e6414bc394adcde7bd037142)`(std::function< void(TEvent &)> process)` | processAndPublish
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a77589f0b36ebd128df58c269bc0468ec) | 

## Members

#### `public inline  `[`DataMultiplexerPublisher`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a8d64c3a740187563f51e57ac31144868)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,std::function< void(const TEvent &, TEvent &)> eventCloner,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` 

[DataMultiplexerPublisher](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher).

#### Parameters
* `container` 

* `name` 

* `eventCloner` optional function used for cloning event that are put in the ring buffer. 

* `ringBuffer`

#### `public inline virtual void `[`processAndPublish`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a1d85e950e6414bc394adcde7bd037142)`(std::function< void(TEvent &)> process)` 

processAndPublish

#### Parameters
* `process`

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerPublisher_1a77589f0b36ebd128df58c269bc0468ec) 

# class `kpsr::high_performance::DataMultiplexerSubscriber` 

```
class kpsr::high_performance::DataMultiplexerSubscriber
  : public kpsr::Subscriber< TEvent >
```  

The [DataMultiplexerSubscriber](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber) class.

2023 Klepsydra Technologies AG

2.0.1

This class is not actually used by the client code, but it is documented due to its close relation to the provider. This class has the same API as most subscribers, but it is not based in the event emmitter as the most of them. Very important to notice is that in the high_performance, only the last event is invoked. This means that faster listeners will process all messages, while slower listener will only process the laster messages while discarding any older ones.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public std::map< std::string, std::shared_ptr< `[`DataMultiplexerListener`](api-undefined.md#classkpsr_1_1high__performance_1_1DataMultiplexerListener)`< TEvent, BufferSize > > > `[`subscriberMap`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a7a2fdcd76e24b4b6b428c502eca4ad30) | subscriberMap
`public inline  `[`DataMultiplexerSubscriber`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a71cd0f73be1f8660ab0916f3cd684081)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` | [DataMultiplexerSubscriber](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber).
`public inline virtual void `[`registerListener`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a9e38e186d19e9fad3bb641cac27374c4)`(std::string name,const std::function< void(const TEvent &)> listener)` | registerListener
`public inline virtual void `[`registerListenerOnce`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a422e42963905506e9cd33cb4660c1d3d)`(const std::function< void(const TEvent &)> listener)` | registerListenerOnce not support at the moment
`public inline virtual void `[`removeListener`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1afe05709e0e30657618499d3221e08bb2)`(std::string name)` | removeListener
`public inline virtual std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a13a911db59e2e047f46fefd546aeb6ac)`(const std::string name)` | getSubscriptionStats getSubscriptionStats retrieves the performance information of the listener.
`typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a8ca065147673dd85b4d4887a8cf94667) | 

## Members

#### `public std::map< std::string, std::shared_ptr< `[`DataMultiplexerListener`](api-undefined.md#classkpsr_1_1high__performance_1_1DataMultiplexerListener)`< TEvent, BufferSize > > > `[`subscriberMap`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a7a2fdcd76e24b4b6b428c502eca4ad30) 

subscriberMap

#### `public inline  `[`DataMultiplexerSubscriber`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a71cd0f73be1f8660ab0916f3cd684081)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,std::string name,`[`RingBuffer`](api-undefined.md#classdisruptor4cpp_1_1ring__buffer)` & ringBuffer)` 

[DataMultiplexerSubscriber](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber).

#### Parameters
* `container` 

* `name` 

* `ringBuffer`

#### `public inline virtual void `[`registerListener`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a9e38e186d19e9fad3bb641cac27374c4)`(std::string name,const std::function< void(const TEvent &)> listener)` 

registerListener

#### Parameters
* `name` 

* `listener`

#### `public inline virtual void `[`registerListenerOnce`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a422e42963905506e9cd33cb4660c1d3d)`(const std::function< void(const TEvent &)> listener)` 

registerListenerOnce not support at the moment

#### Parameters
* `listener`

#### `public inline virtual void `[`removeListener`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1afe05709e0e30657618499d3221e08bb2)`(std::string name)` 

removeListener

#### Parameters
* `name`

#### `public inline virtual std::shared_ptr< `[`SubscriptionStats`](api-kpsr-monitoring.md#structkpsr_1_1SubscriptionStats)` > `[`getSubscriptionStats`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a13a911db59e2e047f46fefd546aeb6ac)`(const std::string name)` 

getSubscriptionStats getSubscriptionStats retrieves the performance information of the listener.

#### Parameters
* `name` 

#### Returns

#### `typedef `[`RingBuffer`](#classkpsr_1_1high__performance_1_1DataMultiplexerSubscriber_1a8ca065147673dd85b4d4887a8cf94667) 

