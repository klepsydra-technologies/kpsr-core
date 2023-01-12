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


class {{ definition.class_name }}Builder {
public:

   {{ definition.class_name }}Builder()
      :
{%- if definition.parent_class %} {{ definition.parent_class.class_name }} ()
      ,
{%- endif %} {{ definition.fields[0].field_name }}()
{%- for field in definition.fields[1:] %}
      , {{ field.field_name }}()
{%- endfor %}
   {}

{% if definition.parent_class.fields %}
{% for field in definition.parent_class.fields %}
   {{ definition.class_name }}Builder & with{{ field.field_name[0].upper() + field.field_name[1:] }}({{ field.field_type }} {{ field.field_name }}) {
     this->{{ field.field_name }} = {{ field.field_name }};
     return *this;
   }
{% endfor %}
{%- endif %}
{% for field in definition.fields %}
   {{ definition.class_name }}Builder & with{{ field.field_name[0].upper() + field.field_name[1:] }}({{ field.field_type }} {{ field.field_name }}) {
     this->{{ field.field_name }} = {{ field.field_name }};
     return *this;
   }
{% endfor %}

   std::shared_ptr<{{ definition.class_name }}> build() {
        std::shared_ptr<{{ definition.class_name }}> ptr(new {{ definition.class_name }}(
{%- if definition.parent_class %}
{%- for field in definition.parent_class.fields %}{{ field.field_name }}, {% endfor %}
{%- endif %}
{%- for field in definition.fields[:-1] %}{{ field.field_name }}, {% endfor %}{{ definition.fields[-1].field_name }}));
        return ptr;
    }

{%- if definition.parent_class.fields %}
{%- for field in definition.parent_class.fields %}
   {{ field.field_type }} {{ field.field_name }};
{%- endfor %}
{%- endif %}
{% for field in definition.fields %}
   {{ field.field_type }} {{ field.field_name }};
{%- endfor %}
};
