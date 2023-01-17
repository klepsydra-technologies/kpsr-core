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

#ifndef {{ definition.define_class_name }}_DDS
#define {{ definition.define_class_name }}_DDS

#include <klepsydra/serialization/mapper.h>
{% for include_file in definition.include_list %}
#include {{ include_file }}
{%- endfor %}

namespace kpsr {
template <>

class Mapper<{{ definition.class_name }}, {{ definition.dds_type }}> {
public:
   void fromMiddleware(const {{ definition.dds_type }} & data, {{ definition.class_name }} & event) {
{%- for field in definition.fields %}
    {%- if not field.is_vector and not field.is_array %}
        {%- if field.is_enum or (field.mapper_name is none) %}
            {%- if field.is_enum %}
      event.{{ field.field_name }} = ({{ field.field_type }}) data.{{ field.field_name }}();
            {%- else %}
      event.{{ field.field_name }} = data.{{ field.field_name }}();
            {%- endif %}
        {%- else %}
      {{ field.mapper_name }}.fromMiddleware(data.{{ field.field_name }}(), event.{{ field.field_name }});
        {%- endif %}
    {%- else %}
      {% if field.is_vector %}
      event.{{ field.field_name }}.resize(data.{{ field.field_name }}().size());
      {%- endif %}
      std::transform(data.{{ field.field_name }}().begin(), data.{{ field.field_name }}().end(), event.{{ field.field_name }}.begin(),
        {%- if field.is_enum or (field.mapper_name is none) %}
           {%- if field.is_enum %}
                     [](const unsigned short int dataItem) {
               {%- if field.use_smart_pointer %}
            return std::make_shared<{{ field.field_type }}>( ({{ field.field_type }}) dataItem);
               {%- else %}
                   {%- if field.use_raw_pointer %}
            return new {{ field.field_type }} ( ({{ field.field_type }}) dataItem);
                   {%- else %}
            return ({{ field.field_type }}) dataItem;
                   {%- endif %}
                {%- endif %}
           {%- else %}
                     [](const {{ field.field_type }} dataItem) {
               {%- if field.use_smart_pointer %}
            return std::make_shared<{{ field.field_type }}>(dataItem);
               {%- else %}
               {%- if field.use_raw_pointer %}
            return new {{ field.field_type }} (dataItem);
                   {%- else %}
            return dataItem;
               {%- endif %}
                {%- endif %}
            {%- endif %}
        {%- else %}
                     [&](const {{ field.dds_type }} dataItem) {
           {%- if field.use_smart_pointer %}
         std::shared_ptr<{{ field.field_type }}> eventData(new {{ field.field_type }}());
         {{ field.mapper_name }}.fromMiddleware(dataItem, * eventData.get());
           {%- else %}
               {%- if field.use_raw_pointer %}
         {{ field.field_type }} * eventData = new {{ field.field_type }}();
         {{ field.mapper_name }}.fromMiddleware(dataItem, * eventData);
               {%- else %}
         {{ field.field_type }} eventData;
         {{ field.mapper_name }}.fromMiddleware(dataItem, eventData);
               {%- endif %}
            {%- endif %}
         return eventData;
        {%- endif %}
      });
    {%- endif %}
{%- endfor %}
   }

   void toMiddleware(const {{ definition.class_name }} &event, {{ definition.dds_type }} &data) {
{%- for field in definition.fields %}
    {%- if not field.is_vector and not field.is_array %}
        {%- if field.is_enum or (field.mapper_name is none) %}
      data.{{ field.field_name }}(event.{{ field.field_name }});
        {%- else %}
      {{ field.mapper_name }}.toMiddleware(event.{{ field.field_name }}, data.{{ field.field_name }}());
        {%- endif %}
    {%- else %}
      {% if field.is_vector %}
      data.{{ field.field_name }}().resize(event.{{ field.field_name }}.size());
      {%- endif %}
      std::transform(event.{{ field.field_name }}.begin(), event.{{ field.field_name }}.end(), data.{{ field.field_name }}().begin(),
         {%- if field.use_smart_pointer %}
                     [&](std::shared_ptr<{{ field.field_type }}> eventData) {
         {%- else %}
            {%- if field.use_raw_pointer %}
                     [&]({{ field.field_type }} * eventData) {
            {%- else %}
                     [&]({{ field.field_type }} eventData) {
            {%- endif %}
        {%- endif %}
        {%- if field.is_enum or (field.mapper_name is none) %}
           {%- if field.use_smart_pointer %}
         return * eventData.get();
           {%- else %}
            {%- if field.use_raw_pointer %}
         return * eventData;
            {%- else %}
         return eventData;
            {%- endif %}
           {%- endif %}
        {%- else %}
         {{ field.dds_type }} dataItem;
         {%- if field.use_smart_pointer %}
         {{ field.mapper_name }}.toMiddleware(* eventData.get(), dataItem);
         {%- else %}
            {%- if field.use_raw_pointer %}
         {{ field.mapper_name }}.toMiddleware(* eventData, dataItem);
            {%- else %}
         {{ field.mapper_name }}.toMiddleware(eventData, dataItem);
            {%- endif %}
        {%- endif %}
         return dataItem;
        {%- endif %}
      });
    {%- endif %}
{%- endfor %}
   }

{%- for mapper_instance in definition.mapper_instances %}
    Mapper<{{ mapper_instance.kpsr_class_name }}, {{ mapper_instance.dds_class_name }}> {{ mapper_instance.mapper_name }};
{%- endfor %}
};
}

#endif // {{ definition.define_class_name }}_DDS
