/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2011  Alexandru Csete, OZ9AEC.

  Authors: Tiago Lobato Gimenes <tlgimenes@gmail.com>

  Comments, questions and bugreports should be submitted via
  http://sourceforge.net/projects/gpredict/
  More details can be found at the project home page:

  http://gpredict.oz9aec.net/

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, visit http://www.fsf.org/
*/

#include "target-sat.h"
#include "sat-log.h"

#ifdef HAVE_CONFIG_H
#  include <build-config.h>
#endif

/* NETWORK */
//#include <sys/types.h>
#ifndef WIN32
#include <sys/socket.h>     /* socket(), connect(), send() */
#include <netinet/in.h>     /* struct sockaddr_in */
#include <arpa/inet.h>      /* htons() */
#include <netdb.h>          /* gethostbyname() */
#else
#include <winsock2.h>
#endif
/* END */

#define FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5

TargetSat *
        new_priority_queue(int num_sats)
{
    TargetSat *t = malloc(sizeof(TargetSat));

    t->numSatToTrack = 0;
    t->sats = malloc(sizeof(int)*num_sats);
    t->minCommunication = malloc(sizeof(float)*num_sats);
    t->priorityQueue = malloc(sizeof(int)*num_sats);

    t->targeting = NULL;
    t->pass = NULL;

    return t;
}

void
        append_elem_priority_queue(TargetSat *target, int i)
{
    int n;

    target->numSatToTrack++;
    n = target->numSatToTrack;

    target->priorityQueue[n-1] = i;
    target->sats[i] = n-1;
}

void
        remove_elem_priority_queue(TargetSat* target, int i)
{
    int j = target->sats[i];
       
    if(j == NOT_IN_PRIORITY_QUEUE){
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Error in priority list"),
                     __FILE__, __LINE__);
    }
    else{
        target->sats[target->priorityQueue[j]] = NOT_IN_PRIORITY_QUEUE;
        for(; j < target->numSatToTrack-1; j++){
            target->priorityQueue[j] = target->priorityQueue[j+1];
            target->sats[target->priorityQueue[j]] = j;
        }
        target->priorityQueue[j] = NOT_IN_PRIORITY_QUEUE;

        target->numSatToTrack--;

        if(target->numSatToTrack == 0){
            target->targeting = NULL;
            target->pass = NULL;
        }
    }
}

void 
       swap_elem_priority_queue(TargetSat* target, int i, int j)
{
    int aux;

    target->sats[target->priorityQueue[i]] = j;
    target->sats[target->priorityQueue[j]] = i;

    aux = target->priorityQueue[j];
    target->priorityQueue[j] = target->priorityQueue[i];
    target->priorityQueue[i] = aux;

}

int get_elem_index_priority_queue(TargetSat *target, int i){
    int j = target->sats[i];

    if(j == NOT_IN_PRIORITY_QUEUE){
        sat_log_log (SAT_LOG_LEVEL_ERROR,
                     _("%s:%d: Error in priority list"),
                     __FILE__, __LINE__);
    }
    return j;
}


