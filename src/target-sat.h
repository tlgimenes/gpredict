/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2009  Alexandru Csete, OZ9AEC.

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
#ifndef __TARGET_SAT_H__
#define __TARGET_SAT_H__ 1

#include "predict-tools.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define NOT_IN_PRIORITY_QUEUE -1

typedef struct _target_sat  TargetSat;

struct _target_sat
{
    int numSatToTrack;              /*!< Number of satellites in the list */
    int *priorityQueue;             /*!< Index of next satellite to track */
    int *sats;                      /*!< Index of corresponding satellite in the priorityQueue */
    float * minCommunication;        /*!< Minimun communication time of the satellite */
    sat_t *targeting;
    pass_t *pass;
};

static TargetSat *new_priority_queue(int num_sats);
static void append_elem_priority_queue(TargetSat* target, int i);
static void remove_elem_priority_queue(TargetSat* target, int i);
static void swap_elem_priority_queue(TargetSat* target, int i, int j);
static int get_elem_index_priority_queue(TargetSat *target, int i);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GTK_ROT_ctrl_H__ */
