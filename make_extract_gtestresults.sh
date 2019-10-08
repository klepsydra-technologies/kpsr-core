#!/bin/sh

echo '#!/bin/bash' > extract_gtestresults.sh

for f in $(find build -iname 'gtestresults.xml'); do
    echo "docker cp \$1:/opt/kpsr-core/$f $(echo $f | sed -e 's@^build/@@g')" >> extract_gtestresults.sh
done

