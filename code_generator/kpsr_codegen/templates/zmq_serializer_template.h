/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef {{ definition.serial_define_class_name }}_ZMQ
#define {{ definition.serial_define_class_name }}_ZMQ

#include <cereal/cereal.hpp>

{%- for include in definition.cereal_include_list %}
#include {{ include }}
{%- endfor %}

{% for include in definition.include_list %}
#include {{ include }}
{%- endfor %}

namespace cereal {
template<class Archive>
void serialize(Archive & archive, {{ definition.class_definition_name }} & event)
{
   archive(
{%- for field_name in definition.field_names[:-1] %}
      cereal::make_nvp("{{ field_name }}", event.{{ field_name }}),
{%- endfor %}
      cereal::make_nvp("{{ definition.field_names[-1] }}", event.{{ definition.field_names[-1] }})
   );
}
}

#endif // {{ definition.serial_define_class_name }}_ZMQ
