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

#define MAX_OBJECT_ARRAY_SIZE       10

/************************************************************************
** Includes
*************************************************************************/
#include "cfe.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include <unistd.h>
//#include "cJSON_Utils.h"

/************************************************************************
** Type Definitions
*************************************************************************/

// 2D bounding-box of YOLO detection
typedef struct {
    double      box_2d_pos_x;
    double      box_2d_pos_y;
    double      box_2d_pos_theta;
    double      box_2d_size_x;
    double      box_2d_size_y;
} box2D;

// 3D bounding-box of YOLO detection
typedef struct {
    double      box_3d_pos_x;
    double      box_3d_pos_y;
    double      box_3d_pos_z;
    double      box_3d_orient_x;
    double      box_3d_orient_y;
    double      box_3d_orient_z;
    double      box_3d_orient_w;
    double      box_3d_size_x;
    double      box_3d_size_y;
    double      box_3d_size_z;
    char        box_3d_frame_id[10];
} box3D;

// TO-DO: Implement once annotation is updated
typedef struct {
    uint32     keypoint_id;
    float      confidence_score;
    float      point_x;
    float      point_y;
    float      point_z;
} keypoint3D;

typedef struct {
    uint32     keypoint_id;
    float      confidence_score;
    float      point_x;
    float      point_y;
} keypoint2D;

typedef struct {
    // Rover ID and confidence score
    uint8       class_id;
    char        class_name[10];
    double      confidenceScore;
    char        object_id[10];

    // 2D bounding box for the rover
    box2D       bounding_box_2d;

    // 3D bounding box for the rover
    box3D       bounding_box_3d;  

    // Image masking data
    // Currently not tracking masked data - may implement later if needed
    //double      mask_height;
    //double      mask_width;
    //double[]    mask_data;

    // TO-DO: Implement once annotation is updated
    // 2D-Keypoints observed in frame
    //keypoint2D  keypoint_2D_listing[10];

    // TO-DO: Implement once annotation is updated
    // 3D-KeyPoints observed in frame
    //char         keyPoint_frame_id[10];
    //keypoint3D   keypoint_3D_listing[10];   

    // The euclidean distance to the rover from the camera
    double      distance;

} rover_state;

// Tracks all the YOLO tracked rovers in the image frame
typedef struct {
    // Time stamp
    uint32          timeStamp_sec;
    uint32          timeStamp_nanoSec;
    uint8           arrayLen;

    // Array containing all the YOLO tracked rovers in the image frame
    rover_state     rovers_array[MAX_OBJECT_ARRAY_SIZE];
} rover_array;

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

int32 sync_fusion_injest(rover_array *rovers, const char *fileIOPath); 
//int32 sync_fusion_injest(rover_array rovers, const char[] fileIOPath);  // Future version with an array of rovers based on ID

#endif /* _sync_node_lib_h_ */

/************************/
/*  End of File Comment */
/************************/
