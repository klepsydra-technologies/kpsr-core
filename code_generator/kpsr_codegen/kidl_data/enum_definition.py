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
class EnumDefinition:
    def __init__(self, enum_name, values, already_exists, include_file):
        self.enum_name = enum_name
        self.values = values
        self.already_exists = already_exists
        self.include_file = include_file

    def __repr__(self):
        return "%s(enum_name=%r, values=%r, already_exists=%r, include_file=%r)" % (
            self.__class__.__name__, self.enum_name, self.values, self.already_exists, self.include_file)
