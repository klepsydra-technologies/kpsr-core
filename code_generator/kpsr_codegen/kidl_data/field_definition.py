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
class FieldDefinition:
    def __init__(self, field_name, field_type, is_vector, use_raw_pointer, use_smart_pointer, is_array, size,
                 is_related_class, is_enum):
        self.field_name = field_name
        self.field_type = field_type
        self.is_vector = is_vector
        self.use_raw_pointer = use_raw_pointer
        self.use_smart_pointer = use_smart_pointer
        self.is_array = is_array
        self.size = size
        self.is_related_class = is_related_class
        self.is_enum = is_enum

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r, is_vector=%r, use_raw_pointer=%r, use_smart_pointer=%r, is_array=%r, size=%r, " \
               "is_related_class=%r, is_enum=%r)" % (
            self.__class__.__name__, self.field_name, self.field_type, self.is_vector, self.use_raw_pointer, self.use_smart_pointer,
            self.is_array, self.size, self.is_related_class, self.is_enum)
