wget $1 -O "lib7842@1.0.0.zip"
prosv5 c purge-template lib7842
prosv5 c f lib7842@1.0.0.zip
prosv5 c u lib7842 --force-apply
rm lib7842@1.0.0.zip
