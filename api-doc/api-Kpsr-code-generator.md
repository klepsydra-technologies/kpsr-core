<p align="right">
  <img width="25%" height="25%"src="../images/klepsydra_logo.jpg">
</p>

# group `Kpsr-code-generator` 

generator package

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public def `[`write_contents_to_file`](#group__kpsr-code-generator_1ga8def10f3636f29f9af861aa305bca674)`(poco_file_name,poco_template_content)`            | Doc for the write_contents_to_file function<br/><br/>More details.
`class `[`kpsr_codegen::generator::generator_engine::Generator`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator) | Doc for the [Generator](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator) class<br/><br/>More dtails.
`class `[`kpsr_codegen::preprocessor::configuration::Configuration`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration) | Doc for the [Configuration](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration) class<br/><br/>Loads the yaml files containing fundamental_types and their mappings to respective middlewares.
`class `[`kpsr_codegen::preprocessor::field_preprocessor::FieldPreprocessor`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor) | Doc for the [FieldPreprocessor](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor) class<br/><br/>Class processes fields in the yaml file.
`class `[`kpsr_codegen::preprocessor::preprocessor::Preprocessor`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor) | Doc for the [Preprocessor](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor) class.
`class `[`kpsr_codegen::processor::dds_idl_processor::DdsIdlProcessor`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor) | Doc for the [DdsIdlProcessor](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor) class<br/><br/>Processes yaml fields to create DDS Idl files.
`class `[`kpsr_codegen::processor::dds_mapper_processor::DdsMapperProcessor`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor) | Doc for the DDSMapperProcessor class<br/><br/>Processes yaml fields to map them to dds compatible types.
`class `[`kpsr_codegen::processor::poco_processor::PocoProcessor`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor) | Doc for the [PocoProcessor](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor) class<br/><br/>Processes yaml fields to create a Plain Old C Object (poco).
`class `[`kpsr_codegen::processor::ros_mapper_processor::RosMapperProcessor`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor) | Doc for the [RosMapperProcessor](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor) class<br/><br/>Processes the yaml data and maps the fields to ROS data types.
`class `[`kpsr_codegen::processor::ros_msg_processor::RosMsgProcessor`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor) | Doc for the [RosMsgProcessor](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor) class<br/><br/>Processes the yaml data to create ROS msg files.
`class `[`kpsr_codegen::processor::zmq_serializer_processor::ZmqSerializerProcessor`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor) | Doc for the [ZmqSerializerProcessor](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor) class<br/><br/>Processes yaml fields to create a serialized files for ZMQ using Cereal.

## Members

#### `public def `[`write_contents_to_file`](#group__kpsr-code-generator_1ga8def10f3636f29f9af861aa305bca674)`(poco_file_name,poco_template_content)` 

Doc for the write_contents_to_file function

More details.

# class `kpsr_codegen::generator::generator_engine::Generator` 

Doc for the [Generator](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator) class

More dtails.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`preprocessor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a92c1f7a5f952f202ab4660840d3a861e) | 
`public  `[`poco_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1acc231f7fe49ce212f8683c982b041c35) | 
`public  `[`ros_mapper_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1add42f3e676a4b91348a3d290f0953d6f) | 
`public  `[`ros_msg_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a8d495666ff76a816249de9988b0e2edc) | 
`public  `[`dds_mapper_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a24273ebe48766c81573e8afa25a23ccf) | 
`public  `[`dds_idl_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a88a32264ad715c6749faef1c922909b6) | 
`public  `[`zmq_serializer_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1adb82976e632bfb235700bf20a94b447c) | 
`public  `[`node_handler_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a98014912f96dd7224fbec84b6d404790) | 
`public  `[`poco_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a9ecee96161e0a1f21c5f7ede1a8a966c) | 
`public  `[`ros_mapper_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aaf1494eadafba078d76a40546c64eb70) | 
`public  `[`ros_msg_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a40def24fbb9a2e33252c937123204f00) | 
`public  `[`dds_mapper_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1ae379f049a57901a325866c424499e9ec) | 
`public  `[`dds_idl_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1acfec96706f9025d4975df3c7c641f864) | 
`public  `[`zmq_serializer_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1abbbec464d7e8add4bd6a18d214b9f529) | 
`public  `[`node_handler_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1ab35315359469d352903ab107cb3a3aac) | 
`public def `[`__init__`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a17875752e89409f47e7a2e7208107d47)`(self,conf_path,template_path)` | The constructor.
`public def `[`render`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1abd01bc6f6bd052171bafe5d485a1f0e8)`(self,input_dir,output_dir,include_path,disable_ros,disable_dds,disable_zmq)` | The render function.
`public def `[`generate_code`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aacd383445b8f4e2ae8946aaea86d10d0)`(self,class_definition_dict,include_path,input_dir,main_kidl_file,output_dir,disable_ros,disable_dds,disable_zmq)` | Generate the code from parsed kidl data.
`public def `[`read_kidl_file`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a514548afbc084b5bd3f25450e477bd48)`(self,input_dir,kidl_file,disable_zmq)` | Read the kidl file.

## Members

#### `public  `[`preprocessor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a92c1f7a5f952f202ab4660840d3a861e) 

#### `public  `[`poco_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1acc231f7fe49ce212f8683c982b041c35) 

#### `public  `[`ros_mapper_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1add42f3e676a4b91348a3d290f0953d6f) 

#### `public  `[`ros_msg_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a8d495666ff76a816249de9988b0e2edc) 

#### `public  `[`dds_mapper_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a24273ebe48766c81573e8afa25a23ccf) 

#### `public  `[`dds_idl_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a88a32264ad715c6749faef1c922909b6) 

#### `public  `[`zmq_serializer_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1adb82976e632bfb235700bf20a94b447c) 

#### `public  `[`node_handler_processor`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a98014912f96dd7224fbec84b6d404790) 

#### `public  `[`poco_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a9ecee96161e0a1f21c5f7ede1a8a966c) 

#### `public  `[`ros_mapper_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aaf1494eadafba078d76a40546c64eb70) 

#### `public  `[`ros_msg_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a40def24fbb9a2e33252c937123204f00) 

#### `public  `[`dds_mapper_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1ae379f049a57901a325866c424499e9ec) 

#### `public  `[`dds_idl_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1acfec96706f9025d4975df3c7c641f864) 

#### `public  `[`zmq_serializer_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1abbbec464d7e8add4bd6a18d214b9f529) 

#### `public  `[`node_handler_template`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1ab35315359469d352903ab107cb3a3aac) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a17875752e89409f47e7a2e7208107d47)`(self,conf_path,template_path)` 

The constructor.

#### Parameters
* `conf_path` The configuration path 

* `template_path` The template path

#### `public def `[`render`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1abd01bc6f6bd052171bafe5d485a1f0e8)`(self,input_dir,output_dir,include_path,disable_ros,disable_dds,disable_zmq)` 

The render function.

#### Parameters
* `input_dir` The input directory 

* `output_dir` The output directory 

* `include_path` 

* `disable_ros` 

* `disable_dds` 

* `disable_zmq`

#### `public def `[`generate_code`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aacd383445b8f4e2ae8946aaea86d10d0)`(self,class_definition_dict,include_path,input_dir,main_kidl_file,output_dir,disable_ros,disable_dds,disable_zmq)` 

Generate the code from parsed kidl data.

#### `public def `[`read_kidl_file`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a514548afbc084b5bd3f25450e477bd48)`(self,input_dir,kidl_file,disable_zmq)` 

Read the kidl file.

Parses the kidl file (written in YAML format).

#### Parameters
* `input_dir` The input folder containing the kidl files 

* `kidl_file` The kidl file to be processed 

* `disable_zmq` Boolean whether to disable zmq export or not.

#### Returns
data from file as a ClassDefinition object

# class `kpsr_codegen::preprocessor::configuration::Configuration` 

Doc for the [Configuration](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration) class

Loads the yaml files containing fundamental_types and their mappings to respective middlewares.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`dds_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1acc3deced56e6aa813cae884540c9bb06) | 
`public  `[`fundamental_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1aa9032778fc4668abe6789f4affa72708) | 
`public  `[`ros_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1afc60d0e8e0c3b21bac1a1326d3f16e49) | 
`public  `[`type_modifiers`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1af1b31203ff388de904fb7cece4232434) | 
`public  `[`type_modifiers_cpp`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a9ae63e9cc4e19e469ef91a6475487125) | 
`public  `[`type_modifiers_dds`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1acf7c50cc6eca626d26f1a372676e4a5b) | 
`public  `[`type_modifiers_ros`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a834ed0f0fe643a2ce65cc93bba658f9a) | 
`public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a21ae1c49a7f324d415ea2ef2350f6e6f)`(self,conf_path)` | 

## Members

#### `public  `[`dds_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1acc3deced56e6aa813cae884540c9bb06) 

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1aa9032778fc4668abe6789f4affa72708) 

#### `public  `[`ros_types`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1afc60d0e8e0c3b21bac1a1326d3f16e49) 

#### `public  `[`type_modifiers`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1af1b31203ff388de904fb7cece4232434) 

#### `public  `[`type_modifiers_cpp`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a9ae63e9cc4e19e469ef91a6475487125) 

#### `public  `[`type_modifiers_dds`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1acf7c50cc6eca626d26f1a372676e4a5b) 

#### `public  `[`type_modifiers_ros`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a834ed0f0fe643a2ce65cc93bba658f9a) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1configuration_1_1Configuration_1a21ae1c49a7f324d415ea2ef2350f6e6f)`(self,conf_path)` 

# class `kpsr_codegen::preprocessor::field_preprocessor::FieldPreprocessor` 

Doc for the [FieldPreprocessor](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor) class

Class processes fields in the yaml file.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`type_modifiers`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1a8ff1c72678489640795dcd4c14663a3a) | 
`public  `[`fundamental_types`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1a3bb38d19807cf7ff7978def96bca6fe4) | 
`public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1af32c030175fc1e09f09c9f40b61e5bed)`(self,type_modifiers,fundamental_types)` | 
`public def `[`process`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1aa5c4b01c6f6b165d3b95f471f87113db)`(self,field,enumeration_dict,is_zmq_enabled)` | Process the fields in the kidl file.

## Members

#### `public  `[`type_modifiers`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1a8ff1c72678489640795dcd4c14663a3a) 

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1a3bb38d19807cf7ff7978def96bca6fe4) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1af32c030175fc1e09f09c9f40b61e5bed)`(self,type_modifiers,fundamental_types)` 

#### `public def `[`process`](#classkpsr__codegen_1_1preprocessor_1_1field__preprocessor_1_1FieldPreprocessor_1aa5c4b01c6f6b165d3b95f471f87113db)`(self,field,enumeration_dict,is_zmq_enabled)` 

Process the fields in the kidl file.

For fundamental data types it does a direct mapping. The matches for pointers (raw or smart) and vectors or arrays are done using regexp. Finally if the data field is none of the above types, it is treated as an enum.

# class `kpsr_codegen::preprocessor::preprocessor::Preprocessor` 

Doc for the [Preprocessor](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor) class.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`fieldProcessor`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a0905bf45302f9434f52f8ee1aeae113a) | 
`public  `[`middlewarePreprocessor`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1ab1c3e9a82ba7e90effd00a706f3dfa7c) | 
`public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a8a4a1e3a5f00e63a17634264d80f3266)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a5082c82ae8482a735f92d79827530834)`(self,class_definition_data,disable_zmq)` | Process the Yaml Object into ClassDefinition object.
`public def `[`process_enum`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a8bf2ae2c4be2ec2f4e2423a4428d7263)`(self,enum)` | Process enum data types.

## Members

#### `public  `[`fieldProcessor`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a0905bf45302f9434f52f8ee1aeae113a) 

#### `public  `[`middlewarePreprocessor`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1ab1c3e9a82ba7e90effd00a706f3dfa7c) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a8a4a1e3a5f00e63a17634264d80f3266)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a5082c82ae8482a735f92d79827530834)`(self,class_definition_data,disable_zmq)` 

Process the Yaml Object into ClassDefinition object.

#### Parameters
* `class_definition_data` python object returned by yaml.load on reading the kidl file. 

* `disable_zmq` Boolean on whether to enable for ZMQ or not.

#### Returns
data from file as a ClassDefinition object

#### `public def `[`process_enum`](#classkpsr__codegen_1_1preprocessor_1_1preprocessor_1_1Preprocessor_1a8bf2ae2c4be2ec2f4e2423a4428d7263)`(self,enum)` 

Process enum data types.

#### Parameters
* `enum` Enum field in the yaml file.

# class `kpsr_codegen::processor::dds_idl_processor::DdsIdlProcessor` 

Doc for the [DdsIdlProcessor](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor) class

Processes yaml fields to create DDS Idl files.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1ac11b6068a3d426c42b434ad6f7326ecf) | 
`public  `[`dds_types`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a1d6dc06b21b4284d2e052a42d4657190) | 
`public  `[`type_modifiers_dds`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a48b11848c7dce769ed8279bb088dc553) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a937e4a226e90312a48fecd39c16ab9b1)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a298e95080941bb6abe6b18f9c9bf4eb6)`(self,class_definition_name,class_definition_dict)` | 
`public def `[`process_includes`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a15b380fbff7f21d638417a69009557e6)`(self,class_definition,class_definition_dict)` | 
`public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1ae4dae8508504f7252c938c870344325f)`(self,field,class_definition_dict,enums)` | 

## Members

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1ac11b6068a3d426c42b434ad6f7326ecf) 

#### `public  `[`dds_types`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a1d6dc06b21b4284d2e052a42d4657190) 

#### `public  `[`type_modifiers_dds`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a48b11848c7dce769ed8279bb088dc553) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a937e4a226e90312a48fecd39c16ab9b1)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a298e95080941bb6abe6b18f9c9bf4eb6)`(self,class_definition_name,class_definition_dict)` 

#### `public def `[`process_includes`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1a15b380fbff7f21d638417a69009557e6)`(self,class_definition,class_definition_dict)` 

#### `public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1dds__idl__processor_1_1DdsIdlProcessor_1ae4dae8508504f7252c938c870344325f)`(self,field,class_definition_dict,enums)` 

# class `kpsr_codegen::processor::dds_mapper_processor::DdsMapperProcessor` 

Doc for the DDSMapperProcessor class

Processes yaml fields to map them to dds compatible types.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`dds_types`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a0d104c5d28b2a067bfe47f6ca1f52d02) | 
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a1bfc420732c6198c3914aaa16160d139) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a366de16525d0dcb6c9bb466bc4d78308)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1ab5eee07d60fb62a75279af53c8066114)`(self,class_definition_name,class_definition_dict,include_path)` | 
`public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a9c2533f3b4fb54dc2e2b8f5ed42167cb)`(self,field,class_definition_dict)` | 

## Members

#### `public  `[`dds_types`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a0d104c5d28b2a067bfe47f6ca1f52d02) 

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a1bfc420732c6198c3914aaa16160d139) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a366de16525d0dcb6c9bb466bc4d78308)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1ab5eee07d60fb62a75279af53c8066114)`(self,class_definition_name,class_definition_dict,include_path)` 

#### `public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1dds__mapper__processor_1_1DdsMapperProcessor_1a9c2533f3b4fb54dc2e2b8f5ed42167cb)`(self,field,class_definition_dict)` 

# class `kpsr_codegen::processor::poco_processor::PocoProcessor` 

Doc for the [PocoProcessor](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor) class

Processes yaml fields to create a Plain Old C Object (poco).

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1acf99f2ce1cf1b403e37bbe66bee02997) | 
`public  `[`type_modifiers_cpp`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a01fc5a87428dacbf56aa47ffeae43f00) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a52a4bf8a0a338f3c12668269b8f4b0bf)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a24084a1e5b07b94e2664a8e451f0490b)`(self,class_definition_name,class_definition_dict,include_path)` | 
`public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1af0876a7a589afbdf3f993e79b18aabbe)`(self,field)` | 
`public def `[`process_enum_data`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a7c577e59a94d5b9807038e4b6469f2d1)`(self,enum)` | 

## Members

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1acf99f2ce1cf1b403e37bbe66bee02997) 

#### `public  `[`type_modifiers_cpp`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a01fc5a87428dacbf56aa47ffeae43f00) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a52a4bf8a0a338f3c12668269b8f4b0bf)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a24084a1e5b07b94e2664a8e451f0490b)`(self,class_definition_name,class_definition_dict,include_path)` 

#### `public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1af0876a7a589afbdf3f993e79b18aabbe)`(self,field)` 

#### `public def `[`process_enum_data`](#classkpsr__codegen_1_1processor_1_1poco__processor_1_1PocoProcessor_1a7c577e59a94d5b9807038e4b6469f2d1)`(self,enum)` 

# class `kpsr_codegen::processor::ros_mapper_processor::RosMapperProcessor` 

Doc for the [RosMapperProcessor](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor) class

Processes the yaml data and maps the fields to ROS data types.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`ros_types`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a135f452fc23edd1aec4db6366406d66f) | 
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a1881048abaed90750b3d1f25360c4435) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a1f5867ab4795b04d787c2fa6b240264b)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a41f5e6e72058816907dfb9abeb739405)`(self,class_definition_name,class_definition_dict,include_path)` | 
`public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1ae990b4db07ffd7de4ffcf0b9e1ece8fe)`(self,field,class_definition_dict)` | 

## Members

#### `public  `[`ros_types`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a135f452fc23edd1aec4db6366406d66f) 

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a1881048abaed90750b3d1f25360c4435) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a1f5867ab4795b04d787c2fa6b240264b)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1a41f5e6e72058816907dfb9abeb739405)`(self,class_definition_name,class_definition_dict,include_path)` 

#### `public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1ros__mapper__processor_1_1RosMapperProcessor_1ae990b4db07ffd7de4ffcf0b9e1ece8fe)`(self,field,class_definition_dict)` 

# class `kpsr_codegen::processor::ros_msg_processor::RosMsgProcessor` 

Doc for the [RosMsgProcessor](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor) class

Processes the yaml data to create ROS msg files.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a6452b392419afdc52d78fa31b399eb6a) | 
`public  `[`ros_types`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a2e1be50e1b0ad6f6c03d3dc711d2ba72) | 
`public  `[`type_modifiers_ros`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a14b964e4f4c33f4d9f19385c8559d76f) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1ac41e095e050a65aacded08ecb5612229)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a1090286f018bb5ce0e08767e85ac872e)`(self,class_definition_name,class_definition_dict)` | 
`public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1aa5fe62f2b74c1fbc371608c033c5dddf)`(self,field,class_definition_dict,enums)` | 

## Members

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a6452b392419afdc52d78fa31b399eb6a) 

#### `public  `[`ros_types`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a2e1be50e1b0ad6f6c03d3dc711d2ba72) 

#### `public  `[`type_modifiers_ros`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a14b964e4f4c33f4d9f19385c8559d76f) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1ac41e095e050a65aacded08ecb5612229)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1a1090286f018bb5ce0e08767e85ac872e)`(self,class_definition_name,class_definition_dict)` 

#### `public def `[`process_field`](#classkpsr__codegen_1_1processor_1_1ros__msg__processor_1_1RosMsgProcessor_1aa5fe62f2b74c1fbc371608c033c5dddf)`(self,field,class_definition_dict,enums)` 

# class `kpsr_codegen::processor::zmq_serializer_processor::ZmqSerializerProcessor` 

Doc for the [ZmqSerializerProcessor](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor) class

Processes yaml fields to create a serialized files for ZMQ using Cereal.

## Summary

 Members                        | Descriptions                                
--------------------------------|---------------------------------------------
`public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1ae5a8eb819e5b2a9cd72a8d8353d665be) | 
`public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1ac07948f3384aa9679479eb17d71e6282)`(self,configuration)` | 
`public def `[`process`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a7041351f235cd3b826fd1ab92e6a9161)`(self,class_definition_name,class_definition_dict,include_path)` | 
`public def `[`process_includes`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a17636cde41c22e40e5e197d0f5f669fb)`(self,class_definition,class_definition_dict,include_path)` | 
`public def `[`process_cereal_includes`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a0b2ee6f28fe5ac6869fed91879b9233f)`(self,class_definition)` | 

## Members

#### `public  `[`fundamental_types`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1ae5a8eb819e5b2a9cd72a8d8353d665be) 

#### `public def `[`__init__`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1ac07948f3384aa9679479eb17d71e6282)`(self,configuration)` 

#### `public def `[`process`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a7041351f235cd3b826fd1ab92e6a9161)`(self,class_definition_name,class_definition_dict,include_path)` 

#### `public def `[`process_includes`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a17636cde41c22e40e5e197d0f5f669fb)`(self,class_definition,class_definition_dict,include_path)` 

#### `public def `[`process_cereal_includes`](#classkpsr__codegen_1_1processor_1_1zmq__serializer__processor_1_1ZmqSerializerProcessor_1a0b2ee6f28fe5ac6869fed91879b9233f)`(self,class_definition)` 

