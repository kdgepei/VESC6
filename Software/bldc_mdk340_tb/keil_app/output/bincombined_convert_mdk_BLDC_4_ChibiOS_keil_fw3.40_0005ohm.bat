C:\Keil\Keil_v506\ARM\ARMCC\bin\fromelf.exe --bincombined --bincombined_padding=1,0x00 --output=./output/project.bin  ./output/project.axf
C:\Keil\Keil_v506\ARM\ARMCC\bin\fromelf.exe --bincombined --bincombined_padding=1,0x00 --output=./output/BLDC_4_ChibiOS.bin  ./output/project.axf

xcopy .\output\project.axf .\output\BLDC_4_ChibiOS_keil_fw3.40_0005ohm /y/q
xcopy .\output\project.hex .\output\BLDC_4_ChibiOS_keil_fw3.40_0005ohm /y/q
xcopy .\output\project.bin .\output\BLDC_4_ChibiOS_keil_fw3.40_0005ohm /y/q

cd %cd%\output\BLDC_4_ChibiOS_keil_fw3.40_0005ohm
call ren.bat

exit
