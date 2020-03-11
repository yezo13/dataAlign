i=1000001;
for x in ./data/*.bin;
do ./bin2pcd --infile $x --outfile ./pcds/$i.pcd;
let i=i+1;
done

cd ./pcds
rename 's/^100/0/' *

