echo "`date`   $0: gestart"

cd `dirname $0`
HIER=`pwd`
[ ! -d ../lst ] && mkdir ../lst
[ ! -d ../plt ] && mkdir ../plt
[ ! -d ../png ] && mkdir ../png

IK=`basename $0 .sh`
TYPE=`echo $IK | awk -F_ '{ print $2 }'`
NODE=`echo $IK | awk -F_ '{ print $3 }'`
PER=`echo $IK | awk -F_ '{ print $4 }'`
echo TYPE=$TYPE NODE=$NODE PER=$PER

php ../php/getmjs20.php $NODE $PER
ls -l ../lst/mjs_${TYPE}_${NODE}_${PER}.lst
ls -l ../lst/mjs_${TYPE}_${NODE}_th_${PER}.lst
ls -l ../lst/mjs_${TYPE}_${NODE}_pm_${PER}.lst
ls -l ../plt/mjs_${TYPE}_${NODE}_th_${PER}.plt
../plt/mjs_${TYPE}_${NODE}_th_${PER}.plt
ls -l ../png/mjs_${TYPE}_${NODE}_th_${PER}.png
ls -l ../plt/mjs_${TYPE}_${NODE}_pm_${PER}.plt
../plt/mjs_${TYPE}_${NODE}_pm_${PER}.plt
ls -l ../png/mjs_${TYPE}_${NODE}_pm_${PER}.png

echo "`date`   $0: gestopt"
echo ""
echo ""
