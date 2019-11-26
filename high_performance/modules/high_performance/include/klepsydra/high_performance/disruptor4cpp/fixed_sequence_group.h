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

/*
Copyright (c) 2015, Alex Man-fui Lee
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of disruptor4cpp nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DISRUPTOR4CPP_FIXED_SEQUENCE_GROUP_H_
#define DISRUPTOR4CPP_FIXED_SEQUENCE_GROUP_H_

#include <climits>
#include <cstdint>
#include <vector>

#include <klepsydra/high_performance/disruptor4cpp/sequence.h>
#include <klepsydra/high_performance/disruptor4cpp/utils/util.h>

namespace disruptor4cpp
{
	template <typename TSequence = sequence>
	class fixed_sequence_group
	{
	public:
		static fixed_sequence_group<TSequence> create(const std::vector<const TSequence*>& sequences)
		{
			fixed_sequence_group<TSequence> group;
			group.sequences_.insert(group.sequences_.begin(), sequences.begin(), sequences.end());
			return group;
		}

		static fixed_sequence_group<TSequence> create(const std::vector<TSequence*>& sequences)
		{
			fixed_sequence_group<TSequence> group;
			group.sequences_.insert(group.sequences_.begin(), sequences.begin(), sequences.end());
			return group;
		}

		static fixed_sequence_group<TSequence> create(const TSequence& sequence)
		{
			fixed_sequence_group<TSequence> group;
			group.sequences_.push_back(&sequence);
			return group;
		}

		fixed_sequence_group() = default;
		~fixed_sequence_group() = default;

		int64_t get() const
		{
			return sequences_.size() == 1 ? sequences_[0]->get()
				: util::get_minimum_sequence(sequences_);
		}

	private:
		std::vector<const TSequence*> sequences_;
	};
}

#endif
