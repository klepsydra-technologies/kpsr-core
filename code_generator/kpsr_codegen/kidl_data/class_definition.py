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
class ClassDefinition:
    def __init__(self, class_name, already_exists, create_builder, include_file, parent_class, enums, middlewares, fields):
        self.class_name = class_name
        self.already_exists = already_exists
        self.create_builder = create_builder
        self.include_file = include_file
        self.parent_class = parent_class
        self.enums = enums
        self.middlewares = middlewares
        self.fields = fields

    def __repr__(self):
        return "%s(class_name=%r, already_exists=%r, create_builder=%r, include_file=%r, parent_class=%r, enums=%r, " \
               "middlewares=%r, fields=%r)" % (
            self.__class__.__name__, self.class_name, self.already_exists, self.create_builder, self.include_file,
            self.parent_class, self.enums, self.middlewares, self.fields)
