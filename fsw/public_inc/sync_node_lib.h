/*************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: sync_node_lib.h
**
** Purpose: 
**   Specification for the sync_node library functions.
**
*************************************************************************/
#ifndef _sync_node_lib_h_
#define _sync_node_lib_h_

/************************************************************************
** Includes
*************************************************************************/
#include "cfe.h"
#include <stdio.h>
#include <stdlib.h>
#include "background/cJSON.h"
#include "background/cJSON_Utils.h"

/************************************************************************
** Type Definitions
*************************************************************************/

typedef struct {
    // TEMP-HOLDER 
    // TO-DO: PLACE VALUES OF YOLO-ROS MESSAGE HERE
} rover_state

/*************************************************************************
** Exported Functions
*************************************************************************/
/************************************************************************/
/** \brief Sync_node Lib Function 
**  
**  \par Description
**        This is a sync_node function
**
**  \par Assumptions, External Events, and Notes:
**        None
**       
**  \returns
**  \retstmt Returns #CFE_SUCCESS \endcode
**  \endreturns
** 
*************************************************************************/
int32 SYNC_NODE_INJEST( void ); 

#endif /* _sync_node_lib_h_ */

/************************/
/*  End of File Comment */
/************************/
