echo "`date`   $0: gestart"
cd `dirname $0`
HIER=`pwd`
IK=`basename $0 .sh`
LST=`echo $IK | awk -F_ '{ printf("%s_%s_%s.lst", $2, $3, $4) }'`
echo LST=$LST

PHYS="`echo $IK | awk -F_ '{ print $3 }'`"
PERIODE="`echo $IK | awk -F_ '{ print $4 }'`"

case $PERIODE in
    04d) STARTTIME="`date --date='-4 days'   '+%Y-%m-%d %H:%M:%S'`" ;;
    02w) STARTTIME="`date --date='-2 weeks'  '+%Y-%m-%d %H:%M:%S'`" ;;
    02m) STARTTIME="`date --date='-2 months' '+%Y-%m-%d %H:%M:%S'`" ;;
    06m) STARTTIME="`date --date='-6 months' '+%Y-%m-%d %H:%M:%S'`" ;;
    02y) STARTTIME="`date --date='-2 years'  '+%Y-%m-%d %H:%M:%S'`" ;;
    *)   STARTTIME="`date --date='-1 days'   '+%Y-%m-%d %H:%M:%S'`" ;;
esac


[ -r ./meetjestad_test.env ] || {
echo "./meetjestad_test.env not found; abort"
exit 1
}
. ./meetjestad_test.env


SQL="SELECT FROM_UNIXTIME(unixtime), temp, humi FROM knmi_th \
WHERE localtijd > '$STARTTIME' ORDER BY unixtime ASC; "

echo $SQL | mysql -N -u$DBUSER -D$DBASE -p$DBPASS | awk '{ printf("%s.%s %.2f %.2f\n", $1, $2, $3, $4) }' > ../lst/$LST
cd ../lst
ls -l ../lst/$LST
echo "`date`   $0: gestopt"
echo ""
echo ""

