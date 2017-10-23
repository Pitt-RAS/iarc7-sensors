#!/bin/bash
#ps script

ps -eo pid:7,user,pri,nice,vsize:10,rssize:10,size:10,state,%cpu:5,%mem:5,cputime,args --sort -%cpu,-%mem,args