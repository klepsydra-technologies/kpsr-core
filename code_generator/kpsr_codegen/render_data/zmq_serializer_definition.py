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
class SerializerDefinition:
    def __init__(self, serial_define_class_name, class_definition_name, include_list, cereal_include_list, field_names):
        self.serial_define_class_name = serial_define_class_name
        self.class_definition_name = class_definition_name
        self.cereal_include_list = cereal_include_list
        self.include_list = include_list
        self.field_names = field_names

    def __repr__(self):
        return "%s(serial_define_class_name=%r, class_definition_name=%r, include_list=%r, cereal_include_list=%r, " \
               "field_names=%r)" % \
               (self.__class__.__name__, self.serial_define_class_name, self.class_definition_name,
                self.include_list, self.cereal_include_list, self.field_names)
