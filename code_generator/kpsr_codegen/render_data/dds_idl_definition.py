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
class IdlFieldDefinition:
    def __init__(self, field_name, field_type):
        self.field_name = field_name
        self.field_type = field_type

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r)" % \
               (self.__class__.__name__, self.field_name, self.field_type)


class IdlDefinition:
    def __init__(self, class_name, module_name, include_list, fields, sequence_fields):
        self.class_name = class_name
        self.module_name = module_name
        self.include_list = include_list
        self.fields = fields
        self.sequence_fields = sequence_fields

    def __repr__(self):
        return "%s(class_name=%r, module_name=%r, include_list=%r, fields=%r, sequence_fields=%r)" % \
               (self.__class__.__name__, self.class_name, self.module_name, self.include_list, self.fields,
                self.sequence_fields)
