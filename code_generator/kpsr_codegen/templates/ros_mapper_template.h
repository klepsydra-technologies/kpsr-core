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

#ifndef {{ definition.define_class_name }}_ROS
#define {{ definition.define_class_name }}_ROS

#include <klepsydra/serialization/mapper.h>
{% for include_file in definition.include_list %}
#include {{ include_file }}
{%- endfor %}

namespace kpsr {
template <>

class Mapper<{{ definition.class_name }}, {{ definition.ros_type }}> {
public:
   void fromMiddleware(const {{ definition.ros_type }} & message, {{ definition.class_name }} & event) {
{%- for field in definition.fields %}
    {%- if not field.is_vector and not field.is_array %}
        {%- if field.is_enum or (field.mapper_name is none) %}
            {%- if field.is_enum %}
      event.{{ field.field_name }} = ({{ field.field_type }}) message.{{ field.field_name }};
            {%- else %}
      event.{{ field.field_name }} = message.{{ field.field_name }};
            {%- endif %}
        {%- else %}
      {{ field.mapper_name }}.fromMiddleware(message.{{ field.field_name }}, event.{{ field.field_name }});
        {%- endif %}
    {%- else %}
      {% if field.is_vector %}
      event.{{ field.field_name }}.resize(message.{{ field.field_name }}.size());
      {%- endif %}
      std::transform(message.{{ field.field_name }}.begin(), message.{{ field.field_name }}.end(), event.{{ field.field_name }}.begin(),
        {%- if field.is_enum or (field.mapper_name is none) %}
           {%- if field.is_enum %}
                     [](const unsigned short int messageData) {
               {%- if field.use_smart_pointer %}
            return std::make_shared<{{ field.field_type }}>( ({{ field.field_type }}) messageData);
               {%- else %}
                   {%- if field.use_raw_pointer %}
            return new {{ field.field_type }} ( ({{ field.field_type }}) messageData);
                   {%- else %}
            return ({{ field.field_type }}) messageData;
                   {%- endif %}
                {%- endif %}
           {%- else %}
                     [](const {{ field.field_type }} messageData) {
               {%- if field.use_smart_pointer %}
            return std::make_shared<{{ field.field_type }}>(messageData);
               {%- else %}
               {%- if field.use_raw_pointer %}
            return new {{ field.field_type }} (messageData);
                   {%- else %}
            return messageData;
               {%- endif %}
                {%- endif %}
            {%- endif %}
        {%- else %}
                     [&](const {{ field.ros_type }} messageData) {
           {%- if field.use_smart_pointer %}
         std::shared_ptr<{{ field.field_type }}> eventData(new {{ field.field_type }}());
         {{ field.mapper_name }}.fromMiddleware(messageData, * eventData.get());
           {%- else %}
               {%- if field.use_raw_pointer %}
         {{ field.field_type }} * eventData = new {{ field.field_type }}();
         {{ field.mapper_name }}.fromMiddleware(messageData, * eventData);
               {%- else %}
         {{ field.field_type }} eventData;
         {{ field.mapper_name }}.fromMiddleware(messageData, eventData);
               {%- endif %}
            {%- endif %}
         return eventData;
        {%- endif %}
      });
    {%- endif %}
{%- endfor %}
   }

   void toMiddleware(const {{ definition.class_name }} &event, {{ definition.ros_type }} &message) {
{%- for field in definition.fields %}
    {%- if not field.is_vector and not field.is_array %}
        {%- if field.is_enum or (field.mapper_name is none) %}
      message.{{ field.field_name }} = event.{{ field.field_name }};
        {%- else %}
      {{ field.mapper_name }}.toMiddleware(event.{{ field.field_name }}, message.{{ field.field_name }});
        {%- endif %}
    {%- else %}
      {% if field.is_vector %}
      message.{{ field.field_name }}.resize(event.{{ field.field_name }}.size());
      {%- endif %}
      std::transform(event.{{ field.field_name }}.begin(), event.{{ field.field_name }}.end(), message.{{ field.field_name }}.begin(),
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
         {{ field.ros_type }} messageData;
         {%- if field.use_smart_pointer %}
         {{ field.mapper_name }}.toMiddleware(* eventData.get(), messageData);
         {%- else %}
            {%- if field.use_raw_pointer %}
         {{ field.mapper_name }}.toMiddleware(* eventData, messageData);
            {%- else %}
         {{ field.mapper_name }}.toMiddleware(eventData, messageData);
            {%- endif %}
        {%- endif %}
         return messageData;
        {%- endif %}
      });
    {%- endif %}
{%- endfor %}
   }

{%- for mapper_instance in definition.mapper_instances %}
    Mapper<{{ mapper_instance.kpsr_class_name }}, {{ mapper_instance.ros_message_name }}> {{ mapper_instance.mapper_name }};
{%- endfor %}
};
}

#endif // {{ definition.define_class_name }}_ROS
