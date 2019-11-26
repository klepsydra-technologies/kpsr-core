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

from configuration import Configuration
from preprocessor import Preprocessor

from ros_mapper_processor import RosMapperProcessor

CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class RosMapperProcessorTest(unittest.TestCase):

    def test_basic(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_mapper_definition = ros_mapper_processor.process('BasicClass', class_definition_dict, '')

        print(ros_mapper_definition)

    def test_related_classes(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

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

        ros_mapper_definition = ros_mapper_processor.process('kpsr::codegen::ClassWithRosMdlwRelatedClasses',
                                                             class_definition_dict, '')

        print(ros_mapper_definition)

    def test_parent_class(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

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

        ros_mapper_definition = ros_mapper_processor.process('kpsr::codegen::ClassWithParentClass',
                                                             class_definition_dict, '')

        print(ros_mapper_definition)


if __name__ == '__main__':
    unittest.main()
