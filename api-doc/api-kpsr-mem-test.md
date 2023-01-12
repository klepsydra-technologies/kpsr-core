<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `kpsr-mem-test` 

This group of classes are meant to be used in the unit testing of the application. They provide facilities to test middleware publishing and subscription along with some other syntatic sugar for testing.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`class `[`kpsr::mem::MemEnv`](#classkpsr_1_1mem_1_1MemEnv) | The [MemEnv](#classkpsr_1_1mem_1_1MemEnv) class.

# class `kpsr::mem::MemEnv` 

```
class kpsr::mem::MemEnv
  : public kpsr::Environment
```  

The [MemEnv](#classkpsr_1_1mem_1_1MemEnv) class.

2023 Klepsydra Technologies AG

2.0.1

Simple in-memory environment manager implemented with std::map instances.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public virtual void `[`getPropertyString`](#classkpsr_1_1mem_1_1MemEnv_1a0b987b7f1d8d48a87bd651812e2424fc)`(const std::string key,std::string & value)` | getPropertyString
`public virtual void `[`getPropertyInt`](#classkpsr_1_1mem_1_1MemEnv_1a2b53fd0ef559e7f43d2a39ec5827e3c3)`(const std::string key,int & value)` | getPropertyInt
`public virtual void `[`getPropertyFloat`](#classkpsr_1_1mem_1_1MemEnv_1ae376f9c1a0d96a0c6f20d6a7d3d531ab)`(const std::string key,float & value)` | getPropertyFloat
`public virtual void `[`getPropertyBool`](#classkpsr_1_1mem_1_1MemEnv_1a1c52d6b3d0931c2b7fc31a494d1a3954)`(const std::string key,bool & value)` | getPropertyBool
`public virtual void `[`setPropertyString`](#classkpsr_1_1mem_1_1MemEnv_1af2bad07622fe285762f40c9be26ac673)`(const std::string key,const std::string value)` | setPropertyString
`public virtual void `[`setPropertyInt`](#classkpsr_1_1mem_1_1MemEnv_1a084566d5a7bd6ed9c2d7f94f9ccec4cc)`(const std::string key,const int & value)` | setPropertyInt
`public virtual void `[`setPropertyFloat`](#classkpsr_1_1mem_1_1MemEnv_1aac4f0a7cc3bfb20d7693ed388fed0bdd)`(const std::string key,const float & value)` | setPropertyFloat
`public virtual void `[`setPropertyBool`](#classkpsr_1_1mem_1_1MemEnv_1ae2f81ecfc2a65c6b0bf90b80c678f9ca)`(const std::string key,const bool & value)` | setPropertyBool
`public inline virtual void `[`persist`](#classkpsr_1_1mem_1_1MemEnv_1aac6ef96c9eb491efa2c98f3ce988f4f5)`()` | persist empty implementation

## Members

#### `public virtual void `[`getPropertyString`](#classkpsr_1_1mem_1_1MemEnv_1a0b987b7f1d8d48a87bd651812e2424fc)`(const std::string key,std::string & value)` 

getPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyInt`](#classkpsr_1_1mem_1_1MemEnv_1a2b53fd0ef559e7f43d2a39ec5827e3c3)`(const std::string key,int & value)` 

getPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyFloat`](#classkpsr_1_1mem_1_1MemEnv_1ae376f9c1a0d96a0c6f20d6a7d3d531ab)`(const std::string key,float & value)` 

getPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`getPropertyBool`](#classkpsr_1_1mem_1_1MemEnv_1a1c52d6b3d0931c2b7fc31a494d1a3954)`(const std::string key,bool & value)` 

getPropertyBool

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyString`](#classkpsr_1_1mem_1_1MemEnv_1af2bad07622fe285762f40c9be26ac673)`(const std::string key,const std::string value)` 

setPropertyString

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyInt`](#classkpsr_1_1mem_1_1MemEnv_1a084566d5a7bd6ed9c2d7f94f9ccec4cc)`(const std::string key,const int & value)` 

setPropertyInt

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyFloat`](#classkpsr_1_1mem_1_1MemEnv_1aac4f0a7cc3bfb20d7693ed388fed0bdd)`(const std::string key,const float & value)` 

setPropertyFloat

#### Parameters
* `key` 

* `value`

#### `public virtual void `[`setPropertyBool`](#classkpsr_1_1mem_1_1MemEnv_1ae2f81ecfc2a65c6b0bf90b80c678f9ca)`(const std::string key,const bool & value)` 

setPropertyBool

#### Parameters
* `key` 

* `value`

#### `public inline virtual void `[`persist`](#classkpsr_1_1mem_1_1MemEnv_1aac6ef96c9eb491efa2c98f3ce988f4f5)`()` 

persist empty implementation

