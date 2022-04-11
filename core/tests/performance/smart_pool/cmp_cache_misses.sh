#!/bin/bash
bin_dir=../../../../build/bin/
size=64
pool_benchmark=BM_SmartPoolCreateAndAcquireVector
no_pool_benchmark=BM_SharedObjectCreateAndResizeVector
for i in {1..10}
do
	for j in {1..2}
	do
		[[ $j = 1 ]] && bm=$pool_benchmark
		[[ $j = 2 ]] && bm=$no_pool_benchmark
		perf_output=$(sudo perf stat -e task-clock,cycles,instructions,cache-references,cache-misses ./$bin_dir/kpsr_smart_pool_core_perf --benchmark_filter=$bm/$size 2>&1 | awk '/cache-misses/ {print $4 " % of cache misses"}')
		[[ $j = 1 ]] && with_pool=$perf_output
		[[ $j = 2 ]] && without_pool=$perf_output
	done
	echo w/pool datasize=$size:
	echo "  " $with_pool
	echo wo/pool datasize=$size:
	echo "  " $without_pool
	size=$((4 * size))
done
