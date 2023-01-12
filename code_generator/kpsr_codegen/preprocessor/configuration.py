# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# -*- coding: utf-8 -*-
import yaml

## Doc for the Configuration class
# \ingroup kpsr-code-generator
#
# Loads the yaml files containing fundamental_types and their mappings
# to respective middlewares.
class Configuration:
    def __init__(self, conf_path):
        with open(conf_path + "/dds_types.yaml", 'r') as stream:
            try:
                self.dds_types = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/fundamental_types.yaml", 'r') as stream:
            try:
                self.fundamental_types = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/ros_types.yaml", 'r') as stream:
            try:
                self.ros_types = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/type_modifiers.yaml", 'r') as stream:
            try:
                self.type_modifiers = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/type_modifiers_cpp.yaml", 'r') as stream:
            try:
                self.type_modifiers_cpp = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/type_modifiers_dds.yaml", 'r') as stream:
            try:
                self.type_modifiers_dds = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
        with open(conf_path + "/type_modifiers_ros.yaml", 'r') as stream:
            try:
                self.type_modifiers_ros = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)
