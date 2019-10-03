#!/bin/bash

echo '#!/bin/bash' > copy_cppcheck.sh
for target in $(make help | grep cppcheck | grep -v '\<all_cppcheck\>' | awk '{print $2}'); do
    mkdir $target
    make $target > $target/cppcheck-result.log 2> $target/cppcheck-result.xml
    sed -i -e 's@/opt/kpsr-core/@@g' $target/cppcheck-result.xml
    echo "mkdir -p $target && docker cp \$1:/opt/kpsr-core/build/$target/cppcheck-result.xml $target" >> copy_cppcheck.sh
done

