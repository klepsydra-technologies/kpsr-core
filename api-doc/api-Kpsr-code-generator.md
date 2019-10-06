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
`public def `[`generate_code`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aacd383445b8f4e2ae8946aaea86d10d0)`(self,class_definition_dict,include_path,input_dir,main_kidl_file,output_dir,disable_ros,disable_dds,disable_zmq)` | 
`public def `[`read_kidl_file`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a514548afbc084b5bd3f25450e477bd48)`(self,input_dir,kidl_file,disable_zmq)` | 

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
* `conf_path` The configuration path (I suppose) 

* `template_path` The template path (I think)

#### `public def `[`render`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1abd01bc6f6bd052171bafe5d485a1f0e8)`(self,input_dir,output_dir,include_path,disable_ros,disable_dds,disable_zmq)` 

The render function.

#### Parameters
* `input_dir` The input directory (I suppose) 

* `output_dir` The output directory (I suppose) 

* `include_path` 

* `disable_ros` 

* `disable_dds` 

* `disable_zmq`

#### `public def `[`generate_code`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1aacd383445b8f4e2ae8946aaea86d10d0)`(self,class_definition_dict,include_path,input_dir,main_kidl_file,output_dir,disable_ros,disable_dds,disable_zmq)` 

#### `public def `[`read_kidl_file`](#classkpsr__codegen_1_1generator_1_1generator__engine_1_1Generator_1a514548afbc084b5bd3f25450e477bd48)`(self,input_dir,kidl_file,disable_zmq)` 

