:Batch file for copying to MBed
:argument 1: .bin output file
:argument 2: MBed root directory
echo 'copying to flash'
xcopy %1 %2
echo 'done copying to flash'
