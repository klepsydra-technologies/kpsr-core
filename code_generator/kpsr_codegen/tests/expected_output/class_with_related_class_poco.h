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

#ifndef CLASS_WITH_RELATED_CLASSES
#define CLASS_WITH_RELATED_CLASSES


#include <string>
#include <vector>

#include "basic_class.h"


class ClassWithRelatedClasses  {
public:

   ClassWithRelatedClasses() {}

   ClassWithRelatedClasses(
          string i,
          BasicClass ii,
          int32 iii,
          std::vector<BasicClass> iv,
          std::vector<int32> v)
      : i(i)
      , ii(ii)
      , iii(iii)
      , iv(iv)
      , v(v)
   {}

   ClassWithRelatedClasses(const ClassWithRelatedClasses & that)
      : i(that.i)
      , ii(that.ii)
      , iii(that.iii)
      , iv(that.iv)
      , v(that.v)
   {}

   void clone(const ClassWithRelatedClasses & that) {
     this->i = that.i;
     this->ii = that.ii;
     this->iii = that.iii;
     this->iv = that.iv;
     this->v = that.v;
   }


   string i;
   BasicClass ii;
   int32 iii;
   std::vector<BasicClass> iv;
};


#endif // CLASS_WITH_RELATED_CLASSES
