for FILE in aanpassingen.txt crontab.????  index.html.???? dblogin.php meetjestad_ro.php
do
    [ -f $FILE ] && echo rm $FILE
done

for DIR in devel import log perf stats zz_archief */lst */log */png */zz_archief */*/zz_archief */*.[0-9][0-9ABC][0-3][0-9] 
do
    [ -d $DIR ] && echo rm -rf $DIR
done


for FILE in */copy*.sh */*/copy*.sh */*/*.lst */*/*.png */*/*/*.png */*.[0-9][0-9ABC][0-9][0-9] */*/*.[0-9][0-9ABC][0-3][0-9] */*/*/*.[0-9][0-9ABC][0-3][0-9] */*.png mysql/*.sed mysql/*.sh
do
    [ -f $FILE ] && echo rm -f $FILE
done


