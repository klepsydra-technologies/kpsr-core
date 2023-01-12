<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-serialization` 

This group of classes contains the API for serializing events to and from middleware.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::Mapper`](#classkpsr_1_1Mapper) | The [Mapper](#classkpsr_1_1Mapper) class.
`class `[`kpsr::Mapper< T, std::vector< unsigned char > >`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1vector_3_01unsigned_01char_01_4_01_4) | The [Mapper](api-kpsr-serialization.md#classkpsr_1_1Mapper)<T, std::vector<unsigned char> > class.

# class `kpsr::Mapper` 

The [Mapper](#classkpsr_1_1Mapper) class.

2023 Klepsydra Technologies AG

2.1.0

Main interface to transform from and to a middleware into the Klepsydra realm. Implementations of this class are only needed for non-memory middlewares like ROS and DDS.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public void `[`fromMiddleware`](#classkpsr_1_1Mapper_1a66191554ca42291e6921acf738fbadef)`(const MddlwClass & message,KpsrClass & event)` | fromMiddleware This method converts a middleware message (e.g., ROS message [geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) into a Klepsydra realm [POCO](https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object) (e.g., kpsr::geometry::PoseEventData)
`public void `[`toMiddleware`](#classkpsr_1_1Mapper_1a2ad05b079e1a4b9321fcfbf9288b070f)`(const KpsrClass & event,MddlwClass & message)` | toMiddleware This method converts a Klepsydra realm [POCO](https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object) (e.g., kpsr::geometry::PoseEventData) into a middleware message (e.g., ROS message [geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

## Members

#### `public void `[`fromMiddleware`](#classkpsr_1_1Mapper_1a66191554ca42291e6921acf738fbadef)`(const MddlwClass & message,KpsrClass & event)` 

fromMiddleware This method converts a middleware message (e.g., ROS message [geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) into a Klepsydra realm [POCO](https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object) (e.g., kpsr::geometry::PoseEventData)

#### Parameters
* `message` 

* `event`

#### `public void `[`toMiddleware`](#classkpsr_1_1Mapper_1a2ad05b079e1a4b9321fcfbf9288b070f)`(const KpsrClass & event,MddlwClass & message)` 

toMiddleware This method converts a Klepsydra realm [POCO](https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object) (e.g., kpsr::geometry::PoseEventData) into a middleware message (e.g., ROS message [geometry_msgs::PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< T, std::vector< unsigned char > >` 

The [Mapper](api-kpsr-serialization.md#classkpsr_1_1Mapper)<T, std::vector<unsigned char> > class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1vector_3_01unsigned_01char_01_4_01_4_1ab72c65e49bea0e74cf51c8216a1aa738)`(const std::vector< unsigned char > & message,T & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1vector_3_01unsigned_01char_01_4_01_4_1a753c08d8190b4b227bba803439034c9c)`(const T & event,std::vector< unsigned char > & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1vector_3_01unsigned_01char_01_4_01_4_1ab72c65e49bea0e74cf51c8216a1aa738)`(const std::vector< unsigned char > & message,T & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1vector_3_01unsigned_01char_01_4_01_4_1a753c08d8190b4b227bba803439034c9c)`(const T & event,std::vector< unsigned char > & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

