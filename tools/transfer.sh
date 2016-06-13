# Script to auto transfer

cd ..

rm `ls | grep .kml`
rm `ls | grep .pyc`

sed -i -e 's/0.0.0.0/192.168.8.1/g' web-ui/index.html
sed -i -e 's/0.0.0.0/192.168.8.1/g' web-ui/js/argiphoto.js
sed -i -e 's/14550/14560/g' mobile-gcs.py
sed -i -e 's/debug.csv/\/data\/mobile-gcs-log.csv/g' mobile-gcs.py

scp -r .* root@192.168.8.1:/root/mobile_gcs_ui/
scp -r web-ui/* root@192.168.8.1:/www/mobile-gcs/web-ui/

sed -i -e 's/192.168.8.1/0.0.0.0/g' web-ui/index.html
sed -i -e 's/192.168.8.1/0.0.0.0/g' web-ui/js/argiphoto.js
sed -i -e 's/14560/14550/g' mobile-gcs.py
sed -i -e 's/\/data\/debug.csv/debug.csv/g' mobile-gcs.py
