#!/bin/bash
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
