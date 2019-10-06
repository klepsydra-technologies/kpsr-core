<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-serialization-cereal` 

This group of classes contains the API for serializing events using Cereal to and from middleware (REST and ZMQ only).

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::Mapper< T, Base >`](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4) | The [Mapper<T, Base>](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4) class.
`class `[`kpsr::Mapper< T, std::string >`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4) | The [Mapper<T, std::string>](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4) class.

# class `kpsr::Mapper< T, Base >` 

The [Mapper<T, Base>](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4) class.

Klepsydra Technologies 2019-2020.

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4_1a69f1e60fa2e0113468fab63ab48dfc9f)`(const Base & message,T & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4_1a7d3c2ddbd0ddddd19e4aa0937d7cf960)`(const T & event,Base & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4_1a69f1e60fa2e0113468fab63ab48dfc9f)`(const Base & message,T & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01Base_01_4_1a7d3c2ddbd0ddddd19e4aa0937d7cf960)`(const T & event,Base & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< T, std::string >` 

The [Mapper<T, std::string>](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4) class.

Klepsydra Technologies 2019-2020.

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4_1add17479ffd4f4ebfa6d34e9968298885)`(const std::string & message,T & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4_1abfdb66bcf228bd19ea1cc4d1e8ba19fa)`(const T & event,std::string & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4_1add17479ffd4f4ebfa6d34e9968298885)`(const std::string & message,T & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01T_00_01std_1_1string_01_4_1abfdb66bcf228bd19ea1cc4d1e8ba19fa)`(const T & event,std::string & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

