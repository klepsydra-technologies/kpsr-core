<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-zmq-composition` 

This group of classes relates exclusively to the assemblying of the application for ZMQ middleware. In Spring terms, the 'wiring' of the application is done using this API. The use of ZMQ is light and minimal intrusion is needed as this modules does not peform any configuration at ZMQ level.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::zmq_mdlw::FromZmqChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) | The [FromZmqChannel](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) class.
`class `[`kpsr::zmq_mdlw::FromZmqMiddlewareProvider`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider) | The [FromZmqMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider) class.
`class `[`kpsr::zmq_mdlw::ToZMQMiddlewareProvider`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider) | The [ToZMQMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider) class.
`class `[`kpsr::zmq_mdlw::ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv) | The [ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv) class.

# class `kpsr::zmq_mdlw::FromZmqChannel` 

The [FromZmqChannel](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) class.

2023 Klepsydra Technologies AG

2.1.0

ZMQ Message to Klepsydra Event channel. It works similarly to other middleware implementations, It has two variations: binary and json deserialization, which is decided via injection.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`FromZmqChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a759de66b5d56fc3c06099c5f9df4b02c)`(`[`ZMQPoller`](api-undefined.md#classkpsr_1_1zmq__mdlw_1_1ZMQPoller)`< U > * zmqPoller)` | [FromZmqChannel](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel).
`public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a8c2e5db307d5a5621323b6011bb65e96)`(std::string topic,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` | registerToTopic
`public inline void `[`unregisterFromTopic`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a3cd7873ebc28c1e55ba636ef70135972)`(std::string topic)` | unregisterFromTopic
`public inline void `[`start`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a8d8b16273cf880c815d0976e5d7cc7fd)`()` | start Launches the polling thread.
`public inline void `[`stop`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a6d7f57dd754f6ec2ee79514345aa4513)`()` | stop

## Members

#### `public inline  `[`FromZmqChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a759de66b5d56fc3c06099c5f9df4b02c)`(`[`ZMQPoller`](api-undefined.md#classkpsr_1_1zmq__mdlw_1_1ZMQPoller)`< U > * zmqPoller)` 

[FromZmqChannel](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel).

#### Parameters
* `zmqPoller` injected object that establish the type of deserialization

#### `public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a8c2e5db307d5a5621323b6011bb65e96)`(std::string topic,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` 

registerToTopic

#### Parameters
* `topic` zmq topic to listen to 

* `internalPublisher` Klepsydra publisher to send the deserialized event to.

#### `public inline void `[`unregisterFromTopic`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a3cd7873ebc28c1e55ba636ef70135972)`(std::string topic)` 

unregisterFromTopic

#### Parameters
* `topic`

#### `public inline void `[`start`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a8d8b16273cf880c815d0976e5d7cc7fd)`()` 

start Launches the polling thread.

#### `public inline void `[`stop`](#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel_1a6d7f57dd754f6ec2ee79514345aa4513)`()` 

stop

# class `kpsr::zmq_mdlw::FromZmqMiddlewareProvider` 

The [FromZmqMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

This class is a wizard to create from zmq channel objects. It simplifies all the wiring for deserialization and instatiation of objects. The following examples ilustrates this: 
```cpp
//  Example of ZMQ socket creation
zmq::context_t context (1);
std::cout << "Collecting updates from weather server...\n" << std::endl;
zmq::socket_t subscriber (context, ZMQ_SUB);

subscriber.connect(url);
subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

// Create an instance FromZmqMiddlewareProvider. Once per application.
kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
_jsomFromZMQProvider->start();

// Create a fromChannel for the above created zmq socket.
kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsomFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<WeatherData>(subscriber, 100);

// Create klepsydra publisher subscriber.
kpsr::mem::SafeQueueMiddlewareProvider<WeatherData> _safeQueueProvider(nullptr, "weatherData", 4, 6, nullptr, nullptr, true);
_safeQueueProvider.start();

// Register the topic. Now the aubscriber in the above created pub/sub pair will start receiving events.
_jsomFromZMQProvider->registerToTopic(topic, _safeQueueProvider.getPublisher());
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< Base > * `[`getBinaryFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1ac9c0301952bfb26642bce8c6ae8478b0)`(zmq::socket_t & subscriber,long pollPeriod)` | getBinaryFromMiddlewareChannel
`public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< std::string > * `[`getJsonFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1aa1e044f8a4dd06a406361777f7cdd4c5)`(zmq::socket_t & subscriber,long pollPeriod)` | getJsonFromMiddlewareChannel
`public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< std::vector< unsigned char > > * `[`getVoidCasterFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1aac1365cd930900f3564a0f8ae91d1c06)`(zmq::socket_t & subscriber,long pollPeriod)` | getJsonFromMiddlewareChannel

## Members

#### `public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< Base > * `[`getBinaryFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1ac9c0301952bfb26642bce8c6ae8478b0)`(zmq::socket_t & subscriber,long pollPeriod)` 

getBinaryFromMiddlewareChannel

#### Parameters
* `subscriber` ZMQ listeing socket 

* `pollPeriod` in milliseconds 

#### Returns
a [FromZmqChannel](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) with binary deserialization

#### `public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< std::string > * `[`getJsonFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1aa1e044f8a4dd06a406361777f7cdd4c5)`(zmq::socket_t & subscriber,long pollPeriod)` 

getJsonFromMiddlewareChannel

#### Parameters
* `subscriber` ZMQ listeing socket 

* `pollPeriod` in milliseconds 

#### Returns
a [FromZmqChannel](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) with json deserialization

#### `public template<>`  <br/>`inline `[`FromZmqChannel`](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel)`< std::vector< unsigned char > > * `[`getVoidCasterFromMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1FromZmqMiddlewareProvider_1aac1365cd930900f3564a0f8ae91d1c06)`(zmq::socket_t & subscriber,long pollPeriod)` 

getJsonFromMiddlewareChannel

#### Parameters
* `subscriber` ZMQ listeing socket 

* `pollPeriod` in milliseconds 

#### Returns
a [FromZmqChannel](api-kpsr-zmq-composition.md#classkpsr_1_1zmq__mdlw_1_1FromZmqChannel) with json deserialization

# class `kpsr::zmq_mdlw::ToZMQMiddlewareProvider` 

The [ToZMQMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

Klepsydra Event to ZMQ Object adapter using Cereal binary serialization. Similarly to other middleware implementations, this class offers the posibility to create a pool of binary serialization object in order to improve performance. This turns out to be quite natural combination with ZMQ. The following example ilustrates the use of this class: 
```cpp
// configure zmq socket to publish
std::string serverUrl = "tcp://*:5556";
std::string topic = "Weather";

zmq::context_t context (1);
zmq::socket_t publisher (context, ZMQ_PUB);
publisher.bind(serverUrl);
publisher.bind("ipc://weather.ipc");

// create provider. One per socket
kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);

// get the instance of the publisher.
kpsr::Publisher<WeatherData> * toZMQPublisher = toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<WeatherData>(topic, 0);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ToZMQMiddlewareProvider`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1aa30e78fe52d8fe3be18ee675f0cae104)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,zmq::socket_t & zmqPublisher)` | [ToZMQMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider).
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getBinaryToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1a38ba6258ea0ce6ba54b4668a7d2dfa82)`(std::string topic,int poolSize)` | getBinaryToMiddlewareChannel
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getJsonToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1afb1f9cc304288b51d60c7b4ee7825f3d)`(std::string topic,int poolSize)` | getJsonToMiddlewareChannel
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getVoidCasterToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1a2ee906cdd85c0f533e65b797eb3d1857)`(std::string topic,int poolSize)` | getVoidCasterToMiddlewareChannel

## Members

#### `public inline  `[`ToZMQMiddlewareProvider`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1aa30e78fe52d8fe3be18ee675f0cae104)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container,zmq::socket_t & zmqPublisher)` 

[ToZMQMiddlewareProvider](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider).

#### Parameters
* `container` 

* `zmqPublisher`

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getBinaryToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1a38ba6258ea0ce6ba54b4668a7d2dfa82)`(std::string topic,int poolSize)` 

getBinaryToMiddlewareChannel

#### Parameters
* `topic` zmq topic 

* `poolSize` object pool size. 0 for no object pool 

#### Returns
binary serializer klepsydra to zmq publisher

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getJsonToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1afb1f9cc304288b51d60c7b4ee7825f3d)`(std::string topic,int poolSize)` 

getJsonToMiddlewareChannel

#### Parameters
* `topic` zmq topic 

* `poolSize` object pool size. 0 for no object pool 

#### Returns
json serializer klepsydra to zmq publisher

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getVoidCasterToMiddlewareChannel`](#classkpsr_1_1zmq__mdlw_1_1ToZMQMiddlewareProvider_1a2ee906cdd85c0f533e65b797eb3d1857)`(std::string topic,int poolSize)` 

getVoidCasterToMiddlewareChannel

#### Parameters
* `topic` zmq topic 

* `poolSize` object pool size. 0 for no object pool 

#### Returns
non serializer klepsydra to zmq publisher

# class `kpsr::zmq_mdlw::ZMQEnv` 

```
class kpsr::zmq_mdlw::ZMQEnv
  : public kpsr::Environment
```  

The [ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv) class.

2023 Klepsydra Technologies AG

2.1.0

ZMQ realm implementation of the Klepsydra [Environment](api-kpsr-application.md#classkpsr_1_1Environment) interface. It also decorate the YALM persistent environment.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public zmq::socket_t & `[`_zmqSubscriber`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ae3966c346f43b20200a9f1291359e7be) | 
`public  `[`ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4f21fad291d18391de62d55d81ac188a)`(const std::string yamlFileName,std::string zmqKey,std::string topicName,int pollPeriod,zmq::socket_t & zmqPublisher,zmq::socket_t & zmqSubscriber)` | [ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv).
`public  `[`ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ae5cdb62265257d1236cae3b9e1bf8ec2)`(YamlEnvironment * yamlEnvironment,zmq::socket_t & zmqPublisher,zmq::socket_t & zmqSubscriber)` | [ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv).
`public  `[`~ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a7dbbeb7e72f48f58e8278f2dbb046cc8)`()` | 
`public virtual void `[`getPropertyString`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a656afd1ab76cf1d10812e1ea0d4e81ae)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a79f7aabc9df235ac89d60bea6d180084)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4bc6043df6f214197487f28b11174a28)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a933a2e2391d013d0e2af78e8ad85e1ed)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1adfb7b0a3f71ce398294994cfc04a5252)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a75f6ba581b8c4eafc5f2e3e71e183b96)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1adcb1f31bcc4f8eb4ffcb072ccc4efb6b)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ac8589617b2b31a5f752fb0f4db228d2f)`(const std::string key,const bool & value)` | setPropertyBool
`public virtual void `[`persist`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1abcc1bb5a0ebc99d5c8ac622bac9f3e55)`()` | persist
`public void `[`updateConfiguration`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4856522b1ffef1b1490857449c0db726)`(std::string configurationData)` | updateConfiguration

## Members

#### `public zmq::socket_t & `[`_zmqSubscriber`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ae3966c346f43b20200a9f1291359e7be) 

#### `public  `[`ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4f21fad291d18391de62d55d81ac188a)`(const std::string yamlFileName,std::string zmqKey,std::string topicName,int pollPeriod,zmq::socket_t & zmqPublisher,zmq::socket_t & zmqSubscriber)` 

[ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv).

#### Parameters
* `yamlFileName` file name to persist to. If empty, no persistent service. 

* `zmqKey` key to identify environment changes messages. 

* `topicName` ZMQ topic where environment data is provided 

* `zmqPublisher` ZMQ specific object 

* `zmqSubscriber` ZMQ specific object

#### `public  `[`ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ae5cdb62265257d1236cae3b9e1bf8ec2)`(YamlEnvironment * yamlEnvironment,zmq::socket_t & zmqPublisher,zmq::socket_t & zmqSubscriber)` 

[ZMQEnv](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv).

#### Parameters
* `yamlEnvironment` 

* `zmqPublisher` 

* `zmqSubscriber`

#### `public  `[`~ZMQEnv`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a7dbbeb7e72f48f58e8278f2dbb046cc8)`()` 

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a656afd1ab76cf1d10812e1ea0d4e81ae)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a79f7aabc9df235ac89d60bea6d180084)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4bc6043df6f214197487f28b11174a28)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a933a2e2391d013d0e2af78e8ad85e1ed)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1adfb7b0a3f71ce398294994cfc04a5252)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a75f6ba581b8c4eafc5f2e3e71e183b96)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1adcb1f31bcc4f8eb4ffcb072ccc4efb6b)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1ac8589617b2b31a5f752fb0f4db228d2f)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`persist`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1abcc1bb5a0ebc99d5c8ac622bac9f3e55)`()` 

persist

This method is used to invoke persistence of the properties. I might be used in cases where the persistance does not happen automatically, for example in the kpsr::YamlEnvironment.

#### `public void `[`updateConfiguration`](#classkpsr_1_1zmq__mdlw_1_1ZMQEnv_1a4856522b1ffef1b1490857449c0db726)`(std::string configurationData)` 

updateConfiguration

#### Parameters
* `configurationData`

