#!/bin/bash
# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

filename=benchmark.mat
[ -e $filename ] && rm -- $filename
bin_dir=../../../../build/bin/

echo '# name: pool' >> $filename
echo '# type: matrix' >> $filename
echo '# rows: 10' >> $filename
echo '# columns: 1' >> $filename
./$bin_dir/kpsr_smart_pool_core_perf --benchmark_filter='BM_SmartPoolCreateAndAcquireVector' 2>&1 | awk '/BM_SmartPoolCreateAndAcquireVector/ {print " " $4}' >> $filename

echo $'\n' >> $filename

echo '# name: no_pool' >> $filename
echo '# type: matrix' >> $filename
echo '# rows: 10' >> $filename
echo '# columns: 1' >> $filename
./$bin_dir/kpsr_smart_pool_core_perf --benchmark_filter='BM_SharedObjectCreateAndResizeVector' 2>&1 | awk '/BM_SharedObjectCreateAndResizeVector/ {print " " $4}' >> $filename

# GNU Octave:
#load benchmark.mat
#plot(1:length(pool), log(pool));hold;plot(1:length(no_pool), log(no_pool))
