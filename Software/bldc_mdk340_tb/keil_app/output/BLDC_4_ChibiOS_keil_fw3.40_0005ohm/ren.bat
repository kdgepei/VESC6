::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 ::::::::::::::::::            NOTE                 :::::::::::::::::::::::::::
 ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

 :: 简要说明：本批处理递归查找当前目录下包含H文件的目录，并将路径名写入HeadDirFile.txt文件中
 ::           路径为相对路径，以批处理所在的目录为起始路径
 :: 使用方法：将批处理文件放到需要查找的文件夹根目录下，然后双击批处理文件，运行即可
 ::               
 ::               运行开始时，提示界面为：processing....
 ::               
 ::               运行结束时，提示界面为：Congratulations! Work is done
 ::
 :: 注意事项：
 :: note 1:   如果需要查找其它后缀文件所在的目录，只需要将note 1对应行的*.h修改为对应名字即可
 ::           example:你需要查找后缀为txt文件所在的目录，只需将*.h改为*.txt就ok啦
 ::
 :: note 2:   本批处理默认输出相对目录名，也可以修改note 2处的代码来输出其它格式的文本
 ::           example：你需要输出完整路径，则需要将note 2中的%%~pi修改为%%i即可
 ::
 :: note 3:   note2 中的参数如下所示
 ::
 ::           %%~I          - 删除任何引号(")，扩展 %I
 ::           %%~fI        - 将 %I 扩展到一个完全合格的路径名
 ::           %%~dI        - 仅将 %I 扩展到一个驱动器号
 ::           %%~pI        - 仅将 %I 扩展到一个路径
 ::           %%~nI        - 仅将 %I 扩展到一个文件名
 ::           %%~xI        - 仅将 %I 扩展到一个文件扩展名
 ::           %%~sI        - 扩展的路径只含有短名
 ::           %%~aI        - 将 %I 扩展到文件的文件属性
 ::           %%~tI        - 将 %I 扩展到文件的日期/时间
 ::           %%~zI        - 将 %I 扩展到文件的大小
 ::           %%~$PATH:I   - 查找列在路径环境变量的目录，并将 %I 扩展
 ::                          到找到的第一个完全合格的名称。如果环境变量名
 ::                          未被定义，或者没有找到文件，此组合键会扩展到
 ::                          空字符串
 ::
 ::           可以组合修饰符来得到多重结果:
 ::
 ::           %%~dpI       - 仅将 %I 扩展到一个驱动器号和路径
 ::           %%~nxI       - 仅将 %I 扩展到一个文件名和扩展名
 ::           %%~fsI       - 仅将 %I 扩展到一个带有短名的完整路径名
 ::           %%~dp$PATH:I - 搜索列在路径环境变量的目录，并将 %I 扩展
 ::                          到找到的第一个驱动器号和路径。
 ::           %%~ftzaI     - 将 %I 扩展到类似输出线路的 DIR
 ::
 :: Hope you enjoy it , thank you ^_^
 ::                                                          Feiyg 2012-05-01



 ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
 ::::::::::::::::::            Below is code      :::::::::::::::::::::::::::::::::::
 ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::


 @ECHO off
 ECHO processing....

 ECHO  ============================================================================    > Return.txt
 ECHO  @brief  list the dir which contains a h file in current root directory                                                                                    >> Return.txt
 ECHO  @author Feiyg                                                                                                                                                      >> Return.txt
 ECHO  @time  %date%----%time%                                                                                                                                       >> Return.txt
 ECHO  ============================================================================   >> Return.txt

 set oldname=project
 set newname=BLDC_4_ChibiOS_keil_fw3.40_0005ohm

 ::note 1
 FOR /R  %%i IN (%newname%.*) DO (
 ::note 2
     IF EXIST %%i (ECHO %%~nxi is deleted.... >> tmp)
     IF EXIST %%i (DEL %%~nxi)
 )

 ::note 1
 FOR /R  %%i IN (%oldname%.*) DO (
 ::note 2
     IF EXIST %%i (ECHO %%~nxi is renamed.... >> tmp)
 )

 for /f "usebackq delims=" %%i in (`dir /s/b/a-d`) do (
  setlocal enabledelayedexpansion
  set var=%%~nxi
  set "var=!var:%oldname%=%newname%!"
  echo %%i |find "ren.bat" >nul ||ren "%%i" "!var!" >nul 2>nul
  endlocal
 )

 FOR /F "delims=" %%i IN (tmp) DO (
 IF NOT DEFINED %%i SET %%i=A & ECHO %%i>>Return.txt)

 DEL tmp

 ECHO Congratulations! Work is done.
 exit