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
import yaml


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
