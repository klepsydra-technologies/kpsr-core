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

sys.path.insert(0, './kpsr_codegen/kidl_data')
sys.path.insert(0, './kpsr_codegen/preprocessor')
sys.path.insert(0, './kpsr_codegen/processor')

from configuration import Configuration
from preprocessor import Preprocessor

from poco_processor import PocoProcessor


CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class PocoProcessorTest(unittest.TestCase):

    def test_basic(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        poco_processor = PocoProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_related_classes.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        poco_definition = poco_processor.process('kpsr::codegen::ClassWithRelatedClasses', class_definition_dict, '')

        print(poco_definition)

    def test_with_parent_class(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        poco_processor = PocoProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_parent_class.yaml"
        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        poco_definition = poco_processor.process('kpsr::codegen::ClassWithParentClass', class_definition_dict, '')

        print(poco_definition)

    def test_with_parent_and_builder(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        poco_processor = PocoProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_parent_class_and_builder.yaml"
        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        poco_definition = poco_processor.process('kpsr::codegen::ClassWithParentClass', class_definition_dict, '')

        print(poco_definition)


if __name__ == '__main__':
    unittest.main()
