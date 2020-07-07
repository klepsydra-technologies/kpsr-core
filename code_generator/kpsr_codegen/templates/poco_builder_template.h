/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

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
