<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-rosstg-composition` 

This group of classes relates exclusively to the assemblying of the application for ROS middleware. In Spring terms, the 'wiring' of the application is done using this API. The use of ROS is light and minimal intrusion is needed as this modules does not peform any configuration at ROS level.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::ros_mdlw::FromRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider) | The [FromRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider) class.
`class `[`kpsr::ros_mdlw::PersistentRosEnv`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv) | The [PersistentRosEnv](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv) class.
`class `[`kpsr::ros_mdlw::RosEnv`](#classkpsr_1_1ros__mdlw_1_1RosEnv) | The [RosEnv](#classkpsr_1_1ros__mdlw_1_1RosEnv) class.
`class `[`kpsr::ros_mdlw::ToRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider) | The [ToRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider) class.

# class `kpsr::ros_mdlw::FromRosMiddlewareProvider` 

The [FromRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.0.1

ROS middleware to Klepsydra adapter or channel. It extends the [event_emitter_subscriber.h](#event__emitter__subscriber_8h_source) class in order to keep track of klepsydra listeners. Its use is very straightforward as per the following example:

```cpp
// Initialize ROS
ros::init(argc, argv, "kpsr_ros_core_test");
ros::NodeHandle nodeHandle;
ros::Rate rate(100);

// Create a from ros provider Klepsydra wizard instance. Once for the whole application.
kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);

// Create a Klepsydra pub/sub pair
kpsr::mem::SafeQueueMiddlewareProvider<std::string> safeQueueProvider(nullptr, "test", 8, 0, nullptr, nullptr, false);
safeQueueProvider.start();

// Obtain a from ros channel and provide the klepsydra publisher
fromRosProvider.registerToTopic<std::string, std_msgs::String>("kpsr_ros_core_test_topic", 1, safeQueueProvider.getPublisher());

// Now, listeners can be registered to the corresponding subscriber.
kpsr::mem::CacheListener<std::string> cacheListener;
safeQueueProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`FromRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider_1add79009dacb67885c430cedd2af444fd)`(ros::NodeHandle & rosNode)` | [FromRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider).
`public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider_1aad40e9963c13669ed51cbf689ceae0cb)`(const char * topicName,int queueSize,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` | registerToTopic

## Members

#### `public inline  `[`FromRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider_1add79009dacb67885c430cedd2af444fd)`(ros::NodeHandle & rosNode)` 

[FromRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider).

#### Parameters
* `rosNode`

#### `public template<>`  <br/>`inline void `[`registerToTopic`](#classkpsr_1_1ros__mdlw_1_1FromRosMiddlewareProvider_1aad40e9963c13669ed51cbf689ceae0cb)`(const char * topicName,int queueSize,`[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * internalPublisher)` 

registerToTopic

#### Parameters
* `topicName` 

* `queueSize` 

* `internalPublisher`

# class `kpsr::ros_mdlw::PersistentRosEnv` 

```
class kpsr::ros_mdlw::PersistentRosEnv
  : public kpsr::Environment
```  

The [PersistentRosEnv](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv) class.

2023 Klepsydra Technologies AG

2.0.1

Ros environment that includes a persistent facility to store the properties in a yaml file. This file can then be read on startup. The way this works is that when a property is set through the Klepsydra [Environment](api-kpsr-application.md#classkpsr_1_1Environment) class, ROS is updated via setParam and also a YAML file as well.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`PersistentRosEnv`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1ae3603b1ced6305a7eb2a3af7713dff6e)`(ros::NodeHandle * nodeHandle,const std::string yamlFileName,const `[`PersitancePolicy`](api-kpsr-rosstg-composition.md#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a644c150cb9166ba724d143feffdaae84)` persitancePolicy)` | [PersistentRosEnv](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv).
`public virtual void `[`getPropertyString`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a3d0d5e3ac92c4067c988afcef0d4e790)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1af36f19e42cf0a3e6f2b1a756ed75dba5)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a55b20161c00bb6df9a0463eeb805ddf7)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1ae7a432af66c6ccb68db4b1a5128bc597)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a9828327d7a6111b1e86a87a2d267e7c9)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a5346c8bd30c680c001e8174a0d575e35)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a880db4ec65397a02538429646194d4ba)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a484fed0642fe57e9fd4338a8255a9989)`(const std::string key,const bool & value)` | setPropertyBool
`public virtual void `[`persist`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a9e1ee2979c62f4d12875218949719c4f)`()` | persist YAML file persist method.
`enum `[`PersitancePolicy`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a644c150cb9166ba724d143feffdaae84) | The PersitancePolicy enum. List of persistance policies to save the YAML file.

## Members

#### `public  `[`PersistentRosEnv`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1ae3603b1ced6305a7eb2a3af7713dff6e)`(ros::NodeHandle * nodeHandle,const std::string yamlFileName,const `[`PersitancePolicy`](api-kpsr-rosstg-composition.md#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a644c150cb9166ba724d143feffdaae84)` persitancePolicy)` 

[PersistentRosEnv](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv).

#### Parameters
* `nodeHandle` 

* `yamlFileName` 

* `persitancePolicy`

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a3d0d5e3ac92c4067c988afcef0d4e790)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1af36f19e42cf0a3e6f2b1a756ed75dba5)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a55b20161c00bb6df9a0463eeb805ddf7)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1ae7a432af66c6ccb68db4b1a5128bc597)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a9828327d7a6111b1e86a87a2d267e7c9)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a5346c8bd30c680c001e8174a0d575e35)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a880db4ec65397a02538429646194d4ba)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a484fed0642fe57e9fd4338a8255a9989)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`persist`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a9e1ee2979c62f4d12875218949719c4f)`()` 

persist YAML file persist method.

#### `enum `[`PersitancePolicy`](#classkpsr_1_1ros__mdlw_1_1PersistentRosEnv_1a644c150cb9166ba724d143feffdaae84) 

 Values                         | Descriptions                                
--------------------------------|---------------------------------------------
NONE            | 
ON_GET            | 
ON_SET            | 
ON_PERSIST            | 

The PersitancePolicy enum. List of persistance policies to save the YAML file.

# class `kpsr::ros_mdlw::RosEnv` 

```
class kpsr::ros_mdlw::RosEnv
  : public kpsr::Environment
```  

The [RosEnv](#classkpsr_1_1ros__mdlw_1_1RosEnv) class.

2023 Klepsydra Technologies AG

2.0.1

An adaptor from ROS Node to Klepsydra [Environment](api-kpsr-application.md#classkpsr_1_1Environment).

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`RosEnv`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a34a3de03406d6b39c16a048455d5595c)`(ros::NodeHandle * nodeHandle)` | [RosEnv](#classkpsr_1_1ros__mdlw_1_1RosEnv).
`public virtual void `[`getPropertyString`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7ef761168aa4f4b9f7f2fb6d563a351c)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a30d46c209355b1562f64a78d3727c19d)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a5a1985b7342aa3508679d4a7e3d6836c)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a1e7fe5a70c9b5327105c8cd6d948f6ce)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7397eab182f6ced1484ebd08d67238ab)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7cc06f9a063ee83cfba640c629f0e17e)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a1a2dd572fb1ee80dac24e46a8c81198b)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1af19dd23576dadbc28e5ab6ad6a2b7d85)`(const std::string key,const bool & value)` | setPropertyBool
`public inline virtual void `[`persist`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a4161fbaf8af5377c7ca768baf35eeaf0)`()` | persist empty implementation

## Members

#### `public  `[`RosEnv`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a34a3de03406d6b39c16a048455d5595c)`(ros::NodeHandle * nodeHandle)` 

[RosEnv](#classkpsr_1_1ros__mdlw_1_1RosEnv).

#### Parameters
* `nodeHandle`

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7ef761168aa4f4b9f7f2fb6d563a351c)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a30d46c209355b1562f64a78d3727c19d)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a5a1985b7342aa3508679d4a7e3d6836c)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a1e7fe5a70c9b5327105c8cd6d948f6ce)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7397eab182f6ced1484ebd08d67238ab)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a7cc06f9a063ee83cfba640c629f0e17e)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a1a2dd572fb1ee80dac24e46a8c81198b)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1af19dd23576dadbc28e5ab6ad6a2b7d85)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public inline virtual void `[`persist`](#classkpsr_1_1ros__mdlw_1_1RosEnv_1a4161fbaf8af5377c7ca768baf35eeaf0)`()` 

persist empty implementation

# class `kpsr::ros_mdlw::ToRosMiddlewareProvider` 

The [ToRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider) class.

2023 Klepsydra Technologies AG

2.0.1

This class is a wizard for cretion of Klepsydra to ROS publishers. Its use is very straightforward as it is shown in the following example: 
```cpp
// Initializing ros
ros::init(argc, argv, "kpsr_ros_core_test");
ros::NodeHandle nodeHandle;
ros::Rate rate(100);

// Get a ros publisher instance
ros::Publisher stringPublisher = nodeHandle.advertise<std_msgs::String>("kpsr_ros_core_test_topic", 1);

// Create the main to ros wizard instance. One for the whole application.
kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

// Obtain the associated klepsydra publisher
kpsr::Publisher<std::string> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<std::string, std_msgs::String>("kpsr_ros_core_test_topic", 1, nullptr, stringPublisher);
```

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline  `[`ToRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider_1a29bda2efb90a0d65c922fcb6a2e2f283)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` | [ToRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider).
`public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getToMiddlewareChannel`](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider_1ae5eaece126408e587941d31397d3f2b0)`(std::string topic,int poolSize,std::function< void(M &)> initializerFunction,ros::Publisher & publisher)` | getToMiddlewareChannel

## Members

#### `public inline  `[`ToRosMiddlewareProvider`](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider_1a29bda2efb90a0d65c922fcb6a2e2f283)`(`[`Container`](api-kpsr-monitoring.md#classkpsr_1_1Container)` * container)` 

[ToRosMiddlewareProvider](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider).

#### Parameters
* `container`

#### `public template<>`  <br/>`inline `[`Publisher`](api-kpsr-application.md#classkpsr_1_1Publisher)`< T > * `[`getToMiddlewareChannel`](#classkpsr_1_1ros__mdlw_1_1ToRosMiddlewareProvider_1ae5eaece126408e587941d31397d3f2b0)`(std::string topic,int poolSize,std::function< void(M &)> initializerFunction,ros::Publisher & publisher)` 

getToMiddlewareChannel

#### Parameters
* `topic` 

* `poolSize` 

* `initializerFunction` 

* `publisher` 

#### Returns

