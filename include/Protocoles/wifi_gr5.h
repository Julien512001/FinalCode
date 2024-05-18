#ifndef WIFI_GR5_H
#define WIFI_GR5_H

#include "../../include/Localization/aruco_gr5.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>

#define PORT 65432
#define BUFFER_SIZE 1024


void wifi(CtrlStruct *cvs);
void handle_client(CtrlStruct *cvs, int new_socket);

#endif