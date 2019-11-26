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
