/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


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
