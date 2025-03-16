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
** File: sync_node_lib.c
**
** Purpose: 
**   Sync_node CFS library
**
*************************************************************************/

/*************************************************************************
** Includes
*************************************************************************/
#include "sync_node_lib.h"
#include "sync_node_lib_version.h"

/*************************************************************************
** Macro Definitions
*************************************************************************/


/*************************************************************************
** Private Function Prototypes
*************************************************************************/
int32 SYNC_NODE_LibInit(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Library Initialization Routine                                  */
/* cFE requires that a library have an initialization routine      */ 
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 SYNC_NODE_LibInit(void)
{
    
    OS_printf ("SYNC_NODE Lib Initialized.  Version %d.%d.%d.%d",
                SYNC_NODE_LIB_MAJOR_VERSION,
                SYNC_NODE_LIB_MINOR_VERSION, 
                SYNC_NODE_LIB_REVISION, 
                SYNC_NODE_LIB_MISSION_REV);
                
    return CFE_SUCCESS;
 
} /* End SYNC_NODE_LibInit */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Sync_node Lib function                                             */ 
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// TO-DO:
// 1.) Create the struct based on the YOLO-ROS message. -
// 2.) File/IO to import JSON file
// 3.) Parse the JSON into rover-struct
// 4.) Calculate the distance from the 3D-bounding box
// 5.) Load the RGB and RGBD images into struct
// 6.) Return the struct 

int32 SYNC_NODE_INJEST(rover_state rover_states, const char[] fileIOPath) 
{
   OS_printf ("SYNC_NODE_Function called\n");
   FILE roverData = OS_open(fileIOPath, OS_READ_ONLY, NULL);

    // TO-DO: Check if pointer is not correct 'file descriptor' int value
    if(roverData == OS_FS_ERR_INVALID_POINTER){
        OS_printf("Invalid file pointer...\n\n");
        return(-1);
    }

    // Opening up the converted JSON file and parsing it into JSON ds.
    CJSON *json = cJSON_Parse(roverData);
    
    // Error checking

    // Parse out each section into Rover-Library-Struct

    

   return(CFE_SUCCESS);
   
} /* End SYNC_NODE_Function */

/************************/
/*  End of File Comment */
/************************/
