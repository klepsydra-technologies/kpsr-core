# Klepsydra Core Modules
# Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# -*- coding: utf-8 -*-

import sys
import unittest
import yaml

sys.path.insert(0, './kidl_data')
sys.path.insert(0, './preprocessor')
sys.path.insert(0, './processor')

from configuration import Configuration
from preprocessor import Preprocessor

from ros_msg_processor import RosMsgProcessor

CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class RosMsgProcessorTest(unittest.TestCase):

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

        print(ros_msg_definition)

    def test_related_classes(self):
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

        print(ros_msg_definition)

    def test_parent_class(self):
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

        print(ros_msg_definition)


if __name__ == '__main__':
    unittest.main()
