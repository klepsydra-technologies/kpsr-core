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

# This code has been automatically generated, manual modification might be inadvertently overridden.

{% if definition.parent_class is not none %}
{%- for field in definition.parent_class.fields %}
{{ field.field_type }} {{ field.field_name }}
{%- endfor %}
{% endif %}
{%- for field in definition.fields %}
{{ field.field_type }} {{ field.field_name }}
{% endfor %}
