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
