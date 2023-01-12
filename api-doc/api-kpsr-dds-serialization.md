<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-dds-serialization` 

This group of classes contains the API for serializing events to and from DDS.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::Mapper< E, kpsr_dds_serialization::LongData >`](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4) | The [Mapper<E, kpsr_dds_serialization::LongData>](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4) class.
`class `[`kpsr::Mapper< bool, kpsr_dds_serialization::BoolData >`](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4) | The [Mapper<bool, kpsr_dds_serialization::BoolData>](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4) class.
`class `[`kpsr::Mapper< unsigned char, kpsr_dds_serialization::OctetData >`](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4) | The [Mapper<unsigned char, kpsr_dds_serialization::OctetData>](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4) class.
`class `[`kpsr::Mapper< int, kpsr_dds_serialization::LongData >`](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4) | The [Mapper<int, kpsr_dds_serialization::LongData>](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4) class.
`class `[`kpsr::Mapper< long, kpsr_dds_serialization::LongLongData >`](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4) | The [Mapper<long, kpsr_dds_serialization::LongLongData>](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4) class.
`class `[`kpsr::Mapper< float, kpsr_dds_serialization::FloatData >`](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4) | The [Mapper<float, kpsr_dds_serialization::FloatData>](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4) class.
`class `[`kpsr::Mapper< double, kpsr_dds_serialization::DoubleData >`](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4) | The [Mapper<double, kpsr_dds_serialization::DoubleData>](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4) class.
`class `[`kpsr::Mapper< std::string, kpsr_dds_serialization::StringData >`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4) | The [Mapper<std::string, kpsr_dds_serialization::StringData>](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4) class.

# class `kpsr::Mapper< E, kpsr_dds_serialization::LongData >` 

The [Mapper<E, kpsr_dds_serialization::LongData>](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4_1abef54f81d5294fb030e2c5c2eb43ece8)`(const kpsr_dds_serialization::LongData & message,E & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4_1a0c1000b98788035d1158ed0492f85bd9)`(const E & event,kpsr_dds_serialization::LongData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4_1abef54f81d5294fb030e2c5c2eb43ece8)`(const kpsr_dds_serialization::LongData & message,E & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01E_00_01kpsr__dds__serialization_1_1LongData_01_4_1a0c1000b98788035d1158ed0492f85bd9)`(const E & event,kpsr_dds_serialization::LongData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< bool, kpsr_dds_serialization::BoolData >` 

The [Mapper<bool, kpsr_dds_serialization::BoolData>](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4_1a000a1871145a52409bf143832657a860)`(const kpsr_dds_serialization::BoolData & message,bool & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4_1abea17b2809e24d5c09e04de49ae42edf)`(const bool & event,kpsr_dds_serialization::BoolData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4_1a000a1871145a52409bf143832657a860)`(const kpsr_dds_serialization::BoolData & message,bool & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01bool_00_01kpsr__dds__serialization_1_1BoolData_01_4_1abea17b2809e24d5c09e04de49ae42edf)`(const bool & event,kpsr_dds_serialization::BoolData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< unsigned char, kpsr_dds_serialization::OctetData >` 

The [Mapper<unsigned char, kpsr_dds_serialization::OctetData>](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4_1a44251ee94f18ec06464c7843c6063a09)`(const kpsr_dds_serialization::OctetData & message,unsigned char & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4_1a45fc5ec321b144d6c164d14dcb3ae407)`(const unsigned char & event,kpsr_dds_serialization::OctetData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4_1a44251ee94f18ec06464c7843c6063a09)`(const kpsr_dds_serialization::OctetData & message,unsigned char & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01unsigned_01char_00_01kpsr__dds__serialization_1_1OctetData_01_4_1a45fc5ec321b144d6c164d14dcb3ae407)`(const unsigned char & event,kpsr_dds_serialization::OctetData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< int, kpsr_dds_serialization::LongData >` 

The [Mapper<int, kpsr_dds_serialization::LongData>](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4_1ac8c24a81bd5d23d9d650daf367aeb0a9)`(const kpsr_dds_serialization::LongData & message,int & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4_1a9ca730625fea458406664c13b4707d1e)`(const int & event,kpsr_dds_serialization::LongData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4_1ac8c24a81bd5d23d9d650daf367aeb0a9)`(const kpsr_dds_serialization::LongData & message,int & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01int_00_01kpsr__dds__serialization_1_1LongData_01_4_1a9ca730625fea458406664c13b4707d1e)`(const int & event,kpsr_dds_serialization::LongData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< long, kpsr_dds_serialization::LongLongData >` 

The [Mapper<long, kpsr_dds_serialization::LongLongData>](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4_1ac8c6369436e576ab83b36c63ea8ec706)`(const kpsr_dds_serialization::LongLongData & message,long & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4_1a427b9f96c16dfb6aacf688c27cde6772)`(const long & event,kpsr_dds_serialization::LongLongData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4_1ac8c6369436e576ab83b36c63ea8ec706)`(const kpsr_dds_serialization::LongLongData & message,long & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01long_00_01kpsr__dds__serialization_1_1LongLongData_01_4_1a427b9f96c16dfb6aacf688c27cde6772)`(const long & event,kpsr_dds_serialization::LongLongData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< float, kpsr_dds_serialization::FloatData >` 

The [Mapper<float, kpsr_dds_serialization::FloatData>](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4_1ad886445037d77119c99ca3b114931e9a)`(const kpsr_dds_serialization::FloatData & message,float & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4_1a76e49af79adc382138d5d6ebf3fc5b5f)`(const float & event,kpsr_dds_serialization::FloatData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4_1ad886445037d77119c99ca3b114931e9a)`(const kpsr_dds_serialization::FloatData & message,float & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01float_00_01kpsr__dds__serialization_1_1FloatData_01_4_1a76e49af79adc382138d5d6ebf3fc5b5f)`(const float & event,kpsr_dds_serialization::FloatData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< double, kpsr_dds_serialization::DoubleData >` 

The [Mapper<double, kpsr_dds_serialization::DoubleData>](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4_1a44e2669d9d681d34d08ab1ac556379b5)`(const kpsr_dds_serialization::DoubleData & message,double & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4_1a645f9ace451cf5abbe14c76ae574228b)`(const double & event,kpsr_dds_serialization::DoubleData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4_1a44e2669d9d681d34d08ab1ac556379b5)`(const kpsr_dds_serialization::DoubleData & message,double & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01double_00_01kpsr__dds__serialization_1_1DoubleData_01_4_1a645f9ace451cf5abbe14c76ae574228b)`(const double & event,kpsr_dds_serialization::DoubleData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

# class `kpsr::Mapper< std::string, kpsr_dds_serialization::StringData >` 

The [Mapper<std::string, kpsr_dds_serialization::StringData>](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4) class.

2023 Klepsydra Technologies AG

2.1.0

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4_1a39202e07de5baa115797208c42ce2407)`(const kpsr_dds_serialization::StringData & message,std::string & event)` | fromMiddleware
`public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4_1a7818983fdd5169222f31e09727b5c488)`(const std::string & event,kpsr_dds_serialization::StringData & message)` | toMiddleware

## Members

#### `public inline void `[`fromMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4_1a39202e07de5baa115797208c42ce2407)`(const kpsr_dds_serialization::StringData & message,std::string & event)` 

fromMiddleware

#### Parameters
* `message` 

* `event`

#### `public inline void `[`toMiddleware`](#classkpsr_1_1Mapper_3_01std_1_1string_00_01kpsr__dds__serialization_1_1StringData_01_4_1a7818983fdd5169222f31e09727b5c488)`(const std::string & event,kpsr_dds_serialization::StringData & message)` 

toMiddleware

#### Parameters
* `event` 

* `message`

