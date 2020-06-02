#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#                            All Rights Reserved.
#
#  This file is subject to the terms and conditions defined in
#  file 'LICENSE.md', which is part of this source code package.
#
#  NOTICE:  All information contained herein is, and remains the property of Klepsydra
#  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
#  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
#  trade secret or copyright law. Dissemination of this information or reproduction of
#  this material is strictly forbidden unless prior written permission is obtained from
#  Klepsydra Technologies GmbH.
#
#****************************************************************************

# -*- coding: utf-8 -*-

## @package generator
# \defgroup kpsr-code-generator
# generator package

import errno
import os
import sys
from os import walk

import yaml
from jinja2 import Environment, FileSystemLoader

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../preprocessor')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../processor')))

from configuration import Configuration

from preprocessor import Preprocessor

from poco_processor import PocoProcessor
from poco_processor import convert_to_lower_case_underscores
from poco_processor import split_namespace_class

from middleware_type import MiddlewareType

from ros_mapper_processor import RosMapperProcessor
from ros_msg_processor import RosMsgProcessor

from dds_mapper_processor import DdsMapperProcessor
from dds_idl_processor import DdsIdlProcessor

from zmq_serializer_processor import ZmqSerializerProcessor

from node_handler_processor import NodeHandlerProcessor

## Doc for the write_contents_to_file function
# \ingroup kpsr-code-generator
#
# More details
def write_contents_to_file(poco_file_name, poco_template_content):
    if not os.path.exists(os.path.dirname(poco_file_name)):
        try:
            os.makedirs(os.path.dirname(poco_file_name))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    poco_file = open(poco_file_name, "w")
    poco_file.write(poco_template_content)
    poco_file.close()


## Doc for the Generator class
# \ingroup kpsr-code-generator
#
# More dtails
class Generator:
    ## The constructor
    # @param conf_path The configuration path
    # @param template_path The template path
    def __init__(self, conf_path, template_path):
        configuration = Configuration(conf_path)
        self.preprocessor = Preprocessor(configuration)
        self.poco_processor = PocoProcessor(configuration)

        self.ros_mapper_processor = RosMapperProcessor(configuration)
        self.ros_msg_processor = RosMsgProcessor(configuration)

        self.dds_mapper_processor = DdsMapperProcessor(configuration)
        self.dds_idl_processor = DdsIdlProcessor(configuration)

        self.zmq_serializer_processor = ZmqSerializerProcessor(configuration)

        self.node_handler_processor = NodeHandlerProcessor(configuration)

        env = Environment(
            loader=FileSystemLoader(template_path)
        )
        self.poco_template = env.get_template('poco_template.h')
        self.ros_mapper_template = env.get_template('ros_mapper_template.h')
        self.ros_msg_template = env.get_template('ros_template.msg')
        self.dds_mapper_template = env.get_template('dds_mapper_template.h')
        self.dds_idl_template = env.get_template('dds_template.idl')
        self.zmq_serializer_template = env.get_template('zmq_serializer_template.h')
        self.node_handler_template = env.get_template('node_handler_template.js')

    ## The render function
    # @param input_dir The input directory
    # @param output_dir The output directory
    # @param include_path
    # @param disable_ros
    # @param disable_dds
    # @param disable_zmq
    def render(self, input_dir, output_dir, include_path, disable_ros, disable_dds, disable_zmq):
        kidl_files = []
        for (dirpath, dirnames, filenames) in walk(input_dir):
            kidl_files.extend(filenames)
            break

        related_classes_dict = {}
        kidl_file_parses = [self.read_kidl_file(input_dir, kidl_file, disable_zmq) for kidl_file in kidl_files]
        class_definitions = [kidl_file_parse if type(kidl_file_parse) is not dict else related_classes_dict.update(kidl_file_parse) for kidl_file_parse in kidl_file_parses ]
        class_definitions = [x for x in class_definitions if x is not None]

        class_definition_dict = {class_definition.class_name: class_definition for class_definition in class_definitions}
        related_classes_dict.update(class_definition_dict)

        [self.generate_code(related_classes_dict, include_path, input_dir, class_name,
                            output_dir, disable_ros, disable_dds, disable_zmq) for class_name in class_definition_dict]

    ## Generate the code from parsed kidl data
    def generate_code(self, class_definition_dict, include_path, input_dir, class_name, output_dir,
                      disable_ros, disable_dds, disable_zmq):
        main_class_definition = class_definition_dict[class_name]
        poco_definition = self.poco_processor.process(main_class_definition.class_name, class_definition_dict,
                                                      include_path)
        if not main_class_definition.already_exists:
            class_name = split_namespace_class(main_class_definition.class_name)[-1]
            poco_file_name = output_dir + "/poco/include/" + include_path + "/" + \
                             convert_to_lower_case_underscores(class_name) + ".h"
            poco_template_content = self.poco_template.render(definition=poco_definition)
            write_contents_to_file(poco_file_name, poco_template_content)

        ros_middleware_definition = main_class_definition.middlewares.get(MiddlewareType.ROS)
        if (not disable_ros) and ros_middleware_definition is not None:
            if not ros_middleware_definition.mapper_include_file:
                ros_mapper_definition = self.ros_mapper_processor.process(main_class_definition.class_name,
                                                                          class_definition_dict, include_path)
                ros_mapper_file_name = output_dir + "/rosstg/include/" + ros_middleware_definition.project_name + "/" \
                                       + convert_to_lower_case_underscores(class_name) + "_ros_mapper.h"
                ros_mapper_template_content = self.ros_mapper_template.render(definition=ros_mapper_definition)
                write_contents_to_file(ros_mapper_file_name, ros_mapper_template_content)

            if not ros_middleware_definition.already_exists:
                ros_msg_definition = self.ros_msg_processor.process(main_class_definition.class_name,
                                                                    class_definition_dict)
                ros_msg_file_name = output_dir + "/rosstg/msg/" + ros_middleware_definition.class_name + ".msg"
                ros_msg_template_content = self.ros_msg_template.render(definition=ros_msg_definition)
                write_contents_to_file(ros_msg_file_name, ros_msg_template_content)

            node_handler_definition = self.node_handler_processor.process(main_class_definition.class_name,
                                                                    class_definition_dict)
            node_handler_file_name = convert_to_lower_case_underscores(
                node_handler_definition.handler_name)+".js"
            node_handler_file_path = output_dir + "/rosstg/node/" + node_handler_file_name
            node_handler_template_content = self.node_handler_template.render(definition=node_handler_definition)
            write_contents_to_file(node_handler_file_path, node_handler_template_content)

        if (not disable_dds) and (main_class_definition.middlewares.get(MiddlewareType.DDS) is not None):
            if not main_class_definition.middlewares.get(MiddlewareType.DDS).mapper_include_file:
                dds_mapper_definition = self.dds_mapper_processor.process(main_class_definition.class_name,
                                                                          class_definition_dict, include_path)
                dds_mapper_file_name = output_dir + "/dds/include/" + include_path + "/dds/" \
                                       + convert_to_lower_case_underscores(class_name) + "_dds_mapper.h"
                dds_mapper_template_content = self.dds_mapper_template.render(definition=dds_mapper_definition)
                write_contents_to_file(dds_mapper_file_name, dds_mapper_template_content)

            if not main_class_definition.middlewares.get(MiddlewareType.DDS).already_exists:
                dds_idl_definition = self.dds_idl_processor.process(main_class_definition.class_name,
                                                                    class_definition_dict)
                dds_idl_file_name = output_dir + "/dds/idl/" + \
                                    convert_to_lower_case_underscores(dds_idl_definition.class_name) + ".idl"
                dds_idl_template_content = self.dds_idl_template.render(definition=dds_idl_definition)
                write_contents_to_file(dds_idl_file_name, dds_idl_template_content)

        if (not disable_zmq) and main_class_definition.middlewares.get(MiddlewareType.ZMQ) is not None:
            if not main_class_definition.middlewares.get(MiddlewareType.ZMQ).serializer_include_file:
                class_name = split_namespace_class(main_class_definition.class_name)[-1]
                zmq_serializer_definition = self.zmq_serializer_processor.process(main_class_definition.class_name,
                                                                                  class_definition_dict, include_path)
                zmq_serializer_file_name = output_dir + "/serialization/include/" + include_path + "/cereal/" \
                                       + convert_to_lower_case_underscores(class_name) + "_serializer.h"
                zmq_serializer_template_content = self.zmq_serializer_template.render(definition=zmq_serializer_definition)
                write_contents_to_file(zmq_serializer_file_name, zmq_serializer_template_content)

    ## Read the kidl file
    #
    # Parses the kidl file (written in YAML format).
    #
    # @param input_dir The input folder containing the kidl files
    # @param kidl_file The kidl file to be processed
    # @param disable_zmq Boolean whether to disable zmq export or not.
    #
    # @return data from file as a ClassDefinition object
    def read_kidl_file(self, input_dir, kidl_file, disable_zmq):
        with open("%s%s" % (input_dir, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
                class_definition = self.preprocessor.process(class_definition_data, disable_zmq)
                return class_definition
            except yaml.YAMLError as exc:
                print(exc)
                return None
