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

#ifndef {{ definition.define_class_name }}_POCO
#define {{ definition.define_class_name }}_POCO
{% for system_include in definition.system_include_list %}
#include {{ system_include }}
{%- endfor %}
{%- for custom_include in definition.custom_include_list %}
#include {{ custom_include }}
{%- endfor %}

{%- if definition.enums %}
{% for enum_name, enum in definition.enums.items() %}
{%- if not enum.already_exists %}
{% for namespace in enum.namespaces %}
namespace {{ namespace }} {
{%- endfor %}
   enum {{ enum.name }} {
{%- for value in enum.values[:-1] %}
      {{ value }},
{%- endfor %}
      {{ enum.values[-1] }}
   };
{%- for namespace in enum.namespaces %}
}
{%- endfor %}
{%- endif %}
{%- endfor %}
{%- endif %}


{% for namespace in definition.namespaces %}
namespace {{ namespace }} {
{%- endfor %}

class {{ definition.class_name }} {% if definition.parent_class %} : public {{ definition.parent_class.class_name }} {% endif %} {
public:

   {{ definition.class_name }}()
      :
{%- if definition.parent_class %} {{ definition.parent_class.class_name }} ()
      ,
{%- endif %} {{ definition.fields[0].field_name }}()
{%- for field in definition.fields[1:] %}
      , {{ field.field_name }}()
{%- endfor %}
   {}

   {{ definition.class_name }}(
{%- if definition.parent_class %}
{%- for field in definition.parent_class.fields %}
       {{ field.field_type }} {{ field.field_name }},
{%- endfor %}
{%- endif %}
{%- for field in definition.fields[:-1] %}
       {{ field.field_type }} {{ field.field_name }},
{%- endfor %}
       {{ definition.fields[-1].field_type }} {{ definition.fields[-1].field_name }})
      :
{%- if definition.parent_class %} {{ definition.parent_class.class_name }} (
{%- for field in definition.parent_class.fields[:-1] %}{{ field.field_name }}, {% endfor %} {{ definition.parent_class.fields[-1].field_name }})
      ,
{%- endif %} {{ definition.fields[0].field_name }}({{ definition.fields[0].field_name }})
{%- for field in definition.fields[1:] %}
      , {{ field.field_name }}({{ field.field_name }})
{%- endfor %}
   {}

   {{ definition.class_name }}(const {{ definition.class_name }} & that)
      :
{%- if definition.parent_class %} {{ definition.parent_class.class_name }} (
{%- for field in definition.parent_class.fields[:-1] %}that.{{ field.field_name }}, {% endfor %} that.{{ definition.parent_class.fields[-1].field_name }})
      ,
{%- endif %} {{ definition.fields[0].field_name }}(that.{{ definition.fields[0].field_name }})

{%- for field in definition.fields[1:] %}
      , {{ field.field_name }}(that.{{ field.field_name }})
{%- endfor %}
   {}

   void clone(const {{ definition.class_name }} & that) {
{%- if definition.parent_class.fields %}
{%- for field in definition.parent_class.fields %}
     this->{{ field.field_name }} = that.{{ field.field_name }};
{%- endfor %}
{%- endif %}
{%- for field in definition.fields %}
     this->{{ field.field_name }} = that.{{ field.field_name }};
{%- endfor %}
   }

{% for field in definition.fields %}
   {{ field.field_type }} {{ field.field_name }};
{%- endfor %}
};

{%- if definition.create_builder %}
{% include 'poco_builder_template.h' %}
{%- endif %}
{%- for namespace in definition.namespaces %}
}
{%- endfor %}

#endif // {{ definition.define_class_name }}_POCO
