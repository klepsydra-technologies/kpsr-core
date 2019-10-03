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

sys.path.insert(0, './kpsr_codegen/kidl_data')
sys.path.insert(0, './kpsr_codegen/preprocessor')
sys.path.insert(0, './kpsr_codegen/processor')

from configuration import Configuration
from preprocessor import Preprocessor

from poco_processor import PocoProcessor

from jinja2 import Template
from jinja2 import Environment, FileSystemLoader


TEMPLATE_PATH = './kpsr_codegen/templates'
CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class Renderer(unittest.TestCase):

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

        env = Environment(
            loader=FileSystemLoader('./kpsr_codegen/templates')
        )
        template = env.get_template('poco_template.h')

        print(template.render(definition=poco_definition))

    def test_with_enums(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        poco_processor = PocoProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_enums.yaml"

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

        poco_definition = poco_processor.process('kpsr::codegen::ClassWithEnums', class_definition_dict, '')

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('poco_template.h')

        print(template.render(definition=poco_definition))

    def test_with_parent(self):
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

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('poco_template.h')

        print(template.render(definition=poco_definition))

    def test_with_builder(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        poco_processor = PocoProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "basic_class_with_builder.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        poco_definition = poco_processor.process('BasicClassWithBuilder', class_definition_dict, '')

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('poco_template.h')

        print(template.render(definition=poco_definition))


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

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('poco_template.h')

        print(template.render(definition=poco_definition))


if __name__ == '__main__':
    unittest.main()
