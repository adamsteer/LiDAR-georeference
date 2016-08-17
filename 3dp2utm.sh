#!/bin/bash

#------------------------------------------------
#make utm coords from RTPP .3dp files using cs2cs
#expects space delimited file with fields:
#time, lat, lon, roll, pitch, heading
#
# These can be produced from .ncom files using 
# RT-post process. 
#
# requires: cs2cs, part of the proj.4 library; awk; pr
#
#------------------------------------------------


if [ -z "$1" ]
 then
  echo "oops - you need to enter an input file, a utm zone, a hemisphere and an output file:"
  echo "#>3dp2utm in.3dp 50 south out.utm"
  echo " a time offset is optional as a fifth argument." 
  exit 1
 fi

infile=$1
#utm zone
zone=$2
#hemisphere
hemisphere=$3
#output file name
outfile=$4
#timeoffset

##check for a time offset
if [ -z "$5" ]
 then
  toffset=0
 else
  toffset=$5
fi

#send time to a temporary file, applying the offset if required
awk -v to=$toffset '{printf "%.5f\n", $1+to}' $infile > t_tmp.arg

# send positions to a temporary file - this will be sent to cs2cs
awk '{print $2,$3,$4}' $infile > llhtmp.arg

# send attitude to a temporary file
awk '{print $5,$6,$7}' $infile > rphtmp.arg

# send error outputs to a temporary file
awk '{print $8,$9,$10,$11,$12,$13}' $infile > errs.arg

##Here, we convert the WGS84 coordinates from RTPP to UTM metres using cs2cs

cs2cs -r -f "%.6f" +proj=latlong +datum=WGS84 +to +proj=utm +zone=$zone +$hemisphere +datum=WGS84 llhtmp.arg > utmtmp.arg

#and finally glue a file back together using pr. Note the double space after -s
pr -m -t -s  t_tmp.arg utmtmp.arg rphtmp.arg errs.arg > tmpfile.arg

#awk to check of we actually have output... to output!
awk '{if (NR!=1) {print}}' tmpfile.arg > $outfile

#cleaning up tabs using sed
sed -i 's/\t/ /g' $outfile

#cleaning up temp files
\rm *.arg

echo "done! thanks."
