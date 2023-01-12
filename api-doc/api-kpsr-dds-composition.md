<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-dds-composition` 

This group of classes relates exclusively to the assemblying of the application for DDS middleware. In Spring terms, the 'wiring' of the application is done using this API. The use of DDS is light and minimal intrusion is needed as this modules does not peform any configuration at DDS level.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::dds_mdlw::DDSEnv`](#classkpsr_1_1dds__mdlw_1_1DDSEnv) | The [DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv) class.
`class `[`kpsr::dds_mdlw::FromDDSMiddlewareProvider`](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider) | The [FromDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider) class.
`class `[`kpsr::dds_mdlw::ToDDSMiddlewareProvider`](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider) | The [ToDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider) class.

# class `kpsr::dds_mdlw::DDSEnv` 

```
class kpsr::dds_mdlw::DDSEnv
  : public kpsr::Environment
```  

The [DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv) class.

2023 Klepsydra Technologies AG

2.1.0

DDS realm implementation of the Klepsydra [Environment](api-kpsr-application.md#classkpsr_1_1Environment) interface. It also decorate the YAML persistent environment. The following is an example of use: 
```cpp
// Create the DDS pub and sub to be used to broadcast configuration
dds::domain::DomainParticipant dp(0);
dds::pub::Publisher pub(dp);
dds::sub::Subscriber sub(dp);

dds::topic::Topic<kpsr_dds_core::DDSEnvironmentData> topic(dp, "kpsrConfigurationTopic");
dds::pub::DataWriter<kpsr_dds_core::DDSEnvironmentData> datawriter(pub, topic);
dds::sub::DataReader<kpsr_dds_core::DDSEnvironmentData> datareader(sub, topic);

// Creation og the environment. Pointer to Environment can now be passed to service, transparently of the actual implementation.
kpsr::dds_mdlw::DDSEnv envSub("./example2.yaml", "example2_conf", &datawriter, &datareader);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`DDSEnv`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1ab46596a5e556a173341baaba8cfcd124)`(const std::string yamlFileName,std::string ddsKey,dds::pub::DataWriter< kpsr_dds_core::DDSEnvironmentData > * dataWriter,dds::sub::DataReader< kpsr_dds_core::DDSEnvironmentData > * dataReader)` | [DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv).
`public  `[`DDSEnv`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a08969144b87bc72515e771cb1b60a0a7)`(YamlEnvironment * yamlEnvironment,dds::pub::DataWriter< kpsr_dds_core::DDSEnvironmentData > * dataWriter,dds::sub::DataReader< kpsr_dds_core::DDSEnvironmentData > * dataReader)` | [DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv).
`public virtual void `[`getPropertyString`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a860ccd5e042962f401aaf4a30bb0ab65)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a3ff5f04bb735ab35d4e96373341c430a)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4affff64160aa94bd718cdf3588f9f87)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a49ca1a00883f73237caf1ed44c3352b4)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a165b020c6dea456274a3b673991b5194)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a0f7e6ef6c51ab0bde720c80af9f48e50)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a39d27ddc9444f1ebd4d648b5d6a229b0)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4d721149b8a49eda5b099b1cdfd9867b)`(const std::string key,const bool & value)` | setPropertyBool
`public virtual void `[`persist`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a250f1fb312a8a9710c099f916d7edefd)`()` | persist
`public void `[`updateConfiguration`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4b71fbb8fd87ee254600ef463892a33c)`(std::string configurationData)` | updateConfiguration

## Members

#### `public  `[`DDSEnv`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1ab46596a5e556a173341baaba8cfcd124)`(const std::string yamlFileName,std::string ddsKey,dds::pub::DataWriter< kpsr_dds_core::DDSEnvironmentData > * dataWriter,dds::sub::DataReader< kpsr_dds_core::DDSEnvironmentData > * dataReader)` 

[DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv).

#### Parameters
* `yamlFileName` file name to persist to. If empty, no persistent service. 

* `ddsKey` key to identify environment changes messages. 

* `dataWriter` 

* `dataReader`

#### `public  `[`DDSEnv`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a08969144b87bc72515e771cb1b60a0a7)`(YamlEnvironment * yamlEnvironment,dds::pub::DataWriter< kpsr_dds_core::DDSEnvironmentData > * dataWriter,dds::sub::DataReader< kpsr_dds_core::DDSEnvironmentData > * dataReader)` 

[DDSEnv](#classkpsr_1_1dds__mdlw_1_1DDSEnv).

#### Parameters
* `yamlEnvironment` 

* `dataWriter` 

* `dataReader`

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a860ccd5e042962f401aaf4a30bb0ab65)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a3ff5f04bb735ab35d4e96373341c430a)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4affff64160aa94bd718cdf3588f9f87)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a49ca1a00883f73237caf1ed44c3352b4)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a165b020c6dea456274a3b673991b5194)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a0f7e6ef6c51ab0bde720c80af9f48e50)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a39d27ddc9444f1ebd4d648b5d6a229b0)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4d721149b8a49eda5b099b1cdfd9867b)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`persist`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a250f1fb312a8a9710c099f916d7edefd)`()` 

persist

This method is used to invoke persistence of the properties. I might be used in cases where the persistance does not happen automatically, for example in the kpsr::YamlEnvironment.

#### `public void `[`updateConfiguration`](#classkpsr_1_1dds__mdlw_1_1DDSEnv_1a4b71fbb8fd87ee254600ef463892a33c)`(std::string configurationData)` 

updateConfiguration

#### Parameters
* `configurationData`

# class `kpsr::dds_mdlw::FromDDSMiddlewareProvider` 

The [FromDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

This class listen to new data coming the DDS. It uses the standard DDS API. The use of this class is very straightforward and just need a reference to a DDS Reader. The following example ilustrates this: 
```cpp
// DDS Reader creation example. Tuning of this are left to the developer.
dds::domain::DomainParticipant dp(0);
dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");
dds::sub::Subscriber sub(dp);
dds::sub::DataReader<tutorial::TempSensorType> dr(sub, topic);

// Creation of the From DDS Provider. It is a one-liner only needed once for all readers.
kpsr::dds_mdlw::FromDDSMiddlewareProvider ddsProvider;

// Create the publisher/subscriber pair that will be channeling the DDS events.
kpsr::mem::SafeQueueMiddlewareProvider<TemperatureData> safeQueueProvider(nullptr, "example", 4, 0, nullptr, nullptr, false);
safeQueueProvider.start();

// Creating a concrete DDS reader and connect it to the publisher side
ddsProvider.registerToTopic("TTempSensor", &dr, true, safeQueueProvider.getPublisher());

// Finally, we pass the associated subscriber to the service that is going to use. Please note that this is
// Transparent of the DDS completely, and yet extremely performing.
TemperatureSubscriberService temperatureSubscriberService(nullptr, safeQueueProvider.getSubscriber());
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider_1a4f4eaa9477ed34be4b34534d76736df1)`(std::string topicName,dds::sub::DataReader< M > * ddsReader,bool useTake,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` | registerToTopic
`public template<>`  <br/>`inline void `[`unregisterFromTopic`](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider_1ae363376f6363160dd005856e5adbdfda)`(std::string topicName,dds::sub::DataReader< M > * ddsReader)` | registerToTopic

## Members

#### `public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider_1a4f4eaa9477ed34be4b34534d76736df1)`(std::string topicName,dds::sub::DataReader< M > * ddsReader,bool useTake,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` 

registerToTopic

#### Parameters
* `topicName` 

* `ddsReader` 

* `useTake` 

* `internalPublisher`

#### `public template<>`  <br/>`inline void `[`unregisterFromTopic`](#classkpsr_1_1dds__mdlw_1_1FromDDSMiddlewareProvider_1ae363376f6363160dd005856e5adbdfda)`(std::string topicName,dds::sub::DataReader< M > * ddsReader)` 

registerToTopic

#### Parameters
* `topicName` 

* `ddsReader` 

* `useTake` 

* `internalPublisher`

# class `kpsr::dds_mdlw::ToDDSMiddlewareProvider` 

The [ToDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.1.0

DDS Middleware provider. It concentrate the creation of publisher and subscriber for a particular topic. It also has advance configuration for performance enhancements. Its use is very straightforward as the following example shows: 
```cpp
// Creation of the DDS writer
dds::domain::DomainParticipant dp(0);
dds::topic::Topic<tutorial::TempSensorType> topic(dp, "TTempSensor");
dds::pub::Publisher pub(dp);
dds::pub::DataWriter<tutorial::TempSensorType> dw(pub, topic);

// The main one-line needed one for the whole process.
kpsr::dds_mdlw::ToDDSMiddlewareProvider provider(nullptr);

// Retrieve the concrete publisher to DDS. Please note the optional object pool tuning params.
kpsr::Publisher<TemperatureData> * temperatureToDDSChannel = provider.getToMiddlewareChannel<TemperatureData, tutorial::TempSensorType>("temperature", 0, nullptr, &dw);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ToDDSMiddlewareProvider`](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider_1a01453d2bf1ec71b33f65fcfc91088ba1)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` | [ToDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider).
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getToMiddlewareChannel`](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider_1acba5dd0fa26715b6b10677cb974a71a3)`(std::string topic,int poolSize,std::function< void(M &)> initializerFunction,dds::pub::DataWriter< M > * dataWriter)` | getToMiddlewareChannel

## Members

#### `public inline  `[`ToDDSMiddlewareProvider`](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider_1a01453d2bf1ec71b33f65fcfc91088ba1)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` 

[ToDDSMiddlewareProvider](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider).

#### Parameters
* `container`

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getToMiddlewareChannel`](#classkpsr_1_1dds__mdlw_1_1ToDDSMiddlewareProvider_1acba5dd0fa26715b6b10677cb974a71a3)`(std::string topic,int poolSize,std::function< void(M &)> initializerFunction,dds::pub::DataWriter< M > * dataWriter)` 

getToMiddlewareChannel

#### Parameters
* `topic` 

* `poolSize` 

* `initializerFunction` 

* `dataWriter` 

#### Returns

