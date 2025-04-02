#!/bin/bash
mypath=$(dirname $0)

function getdir(){
    for element in `ls $1`
    do  
        dir_or_file=$1"/"$element
        if [ -f $dir_or_file ]
        then
            #找到了Origin文件夹内的文件
            OriFile=$dir_or_file
            FirFile="/root/"$element;
            #echo $OriFile,$FirFile
            #如果目标文件存在，则对比文件是否相同
            if [ -f $FirFile ]
            then
                diff $OriFile $FirFile > /dev/null
                if [ $? != 0 ]
                then
                    cp -f $OriFile $FirFile
                    echo Update wsl file $OriFile
                fi
            else
                #如果不存在
                cp -f $OriFile $FirFile
                echo Update wsl file $OriFile
            fi
        fi  
    done
}

getdir ${mypath}/wsl