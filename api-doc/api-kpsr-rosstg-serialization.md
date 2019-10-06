<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-rosstg-serialization` 

This group of classes contains the API for serializing events to and from ROS.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::Mapper< E, std_msgs::Int32 >`](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4) | The [Mapper<E, std_msgs::Int32>](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4) class.
`class `[`kpsr::Mapper< bool, std_msgs::Bool >`](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4) | The [Mapper<bool, std_msgs::Bool>](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4) class.
`class `[`kpsr::Mapper< int, std_msgs::Int32 >`](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4) | The [Mapper<int, std_msgs::Int32>](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4) class.
`class `[`kpsr::Mapper< long, std_msgs::Int64 >`](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4) | The [Mapper<long, std_msgs::Int64>](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4) class.
`class `[`kpsr::Mapper< float, std_msgs::Float32 >`](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4) | The [Mapper<float, std_msgs::Float32>](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4) class.
`class `[`kpsr::Mapper< std::string, std_msgs::String >`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4) | The [Mapper<std::string, std_msgs::String>](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4) class.

# class `kpsr::Mapper< E, std_msgs::Int32 >` 

The [Mapper<E, std_msgs::Int32>](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline virtual void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4_1ae7998d2920c2fcec4783161d00e1e238)`(const std_msgs::Int32 & message,E & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4_1a3e7c6f2157ac00673c89de76d3040132)`(const E & event,std_msgs::Int32 & message)` | toMiddleware

## Members

#### `public inline virtual void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4_1ae7998d2920c2fcec4783161d00e1e238)`(const std_msgs::Int32 & message,E & event)` 

fromMiddleware

#### Parameters
* `environment` 

* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01std__msgs_1_1Int32_01_4_1a3e7c6f2157ac00673c89de76d3040132)`(const E & event,std_msgs::Int32 & message)` 

toMiddleware

#### Parameters
* `environment` 

* `event` 

* `message`

# class `kpsr::Mapper< bool, std_msgs::Bool >` 

The [Mapper<bool, std_msgs::Bool>](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4_1a7cebdcce30b48160669e0e6012c6cd38)`(const std_msgs::Bool & message,bool & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4_1a7886f19ee8f3b520fa5251ba5e247c72)`(const bool & event,std_msgs::Bool & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4_1a7cebdcce30b48160669e0e6012c6cd38)`(const std_msgs::Bool & message,bool & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01std__msgs_1_1Bool_01_4_1a7886f19ee8f3b520fa5251ba5e247c72)`(const bool & event,std_msgs::Bool & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< int, std_msgs::Int32 >` 

The [Mapper<int, std_msgs::Int32>](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4_1a80394fc5eb26a56323c046d72e0dd7b9)`(const std_msgs::Int32 & message,int & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4_1a1756b42e7a5afcb378de776da687c3e7)`(const int & event,std_msgs::Int32 & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4_1a80394fc5eb26a56323c046d72e0dd7b9)`(const std_msgs::Int32 & message,int & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01std__msgs_1_1Int32_01_4_1a1756b42e7a5afcb378de776da687c3e7)`(const int & event,std_msgs::Int32 & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< long, std_msgs::Int64 >` 

The [Mapper<long, std_msgs::Int64>](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4_1a76d947312d5d4c1acaf0f5b555f42818)`(const std_msgs::Int64 & message,long & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4_1abce2fe733031d9cdab48af9196f62383)`(const long & event,std_msgs::Int64 & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4_1a76d947312d5d4c1acaf0f5b555f42818)`(const std_msgs::Int64 & message,long & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01std__msgs_1_1Int64_01_4_1abce2fe733031d9cdab48af9196f62383)`(const long & event,std_msgs::Int64 & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< float, std_msgs::Float32 >` 

The [Mapper<float, std_msgs::Float32>](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4_1a29173beb525e3bf560aaf23aa8de4b86)`(const std_msgs::Float32 & message,float & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4_1a259951e50737294cab3fa08254516c93)`(const float & event,std_msgs::Float32 & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4_1a29173beb525e3bf560aaf23aa8de4b86)`(const std_msgs::Float32 & message,float & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01std__msgs_1_1Float32_01_4_1a259951e50737294cab3fa08254516c93)`(const float & event,std_msgs::Float32 & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< std::string, std_msgs::String >` 

The [Mapper<std::string, std_msgs::String>](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4) class.

Klepsydra Technologies 2019-2020.

2.0.1

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4_1a69aa55bb3e45bed8afe51191dac0b535)`(const std_msgs::String & message,std::string & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4_1a62d41167946ecada5f033fe1f1c3cd36)`(const std::string & event,std_msgs::String & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4_1a69aa55bb3e45bed8afe51191dac0b535)`(const std_msgs::String & message,std::string & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01std__msgs_1_1String_01_4_1a62d41167946ecada5f033fe1f1c3cd36)`(const std::string & event,std_msgs::String & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

