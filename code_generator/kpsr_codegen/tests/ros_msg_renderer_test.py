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

import sys
import unittest
import yaml

sys.path.insert(0, './kidl_data')
sys.path.insert(0, './preprocessor')
sys.path.insert(0, './processor')
sys.path.insert(0, './renderer')

from configuration import Configuration
from preprocessor import Preprocessor

from ros_msg_processor import RosMsgProcessor

from jinja2 import Environment, FileSystemLoader, Template

TEMPLATE_PATH = './kpsr_codegen/templates'
CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class RosMsgRenderer(unittest.TestCase):

    def test_basic(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_msg_processor = RosMsgProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_msg_definition = ros_msg_processor.process('BasicClass', class_definition_dict)

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_template.msg')

        print(template.render(definition=ros_msg_definition))

    def test_with_related_class(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_msg_processor = RosMsgProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_ros_mdlw_and_related_classes.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_msg_definition = ros_msg_processor.process('kpsr::codegen::ClassWithRosMdlwRelatedClasses', class_definition_dict)

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_template.msg')

        print(template.render(definition=ros_msg_definition))

    def test_with_parent_class(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_msg_processor = RosMsgProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_ros_mdlw_and_parent_class.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_msg_definition = ros_msg_processor.process('kpsr::codegen::ClassWithParentClass', class_definition_dict)

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_template.msg')

        print(template.render(definition=ros_msg_definition))


if __name__ == '__main__':
    unittest.main()
