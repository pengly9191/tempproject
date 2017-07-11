#pragma once

#include<stdio.h>
#include<stdlib.h>

#define TAG "POINT"

#define LOGV(x, ...)   	fprintf(stdout,"%s:  " x "\n",TAG, ##__VA_ARGS__);

#define LOGPP(x, ...) LOGV("%s(%d)  " x,  __FUNCTION__,__LINE__, ##__VA_ARGS__)
