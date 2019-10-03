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
