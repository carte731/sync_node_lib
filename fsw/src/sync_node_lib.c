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

// Opens the JSON file and parses it
cJSON parseJSON(const char[] fileIOPath){
    // Opens the YOLO-JSON file 
    FILE *roverData = OS_open(fileIOPath + "/trackOutput.json", OS_READ_ONLY, NULL); 

    // TO-DO: Check if pointer is not correct 'file descriptor' int value
    if(roverData == OS_FS_ERR_INVALID_POINTER){
        OS_printf("Invalid file or file doesn't exist...");
        return(-2);
    }
  
    // Buffer space for raeding the file pointer
    char input_buffer[2048]; 

    // OSAL FILE-IO operations - copies the file into buffer space
    int len = OS_read(roverData, input_buffer, sizeof(input_buffer));

    // Uses OSAL to close the file pointer
    int32 results = OS_close(roverData); 

    // Check if the file was closed correctly
    if(results == OS_FS_ERROR) {
        OS_printf("Error in closing the file space...");
        return(-2)
    }
  
    // parse the JSON data into the data-structure
    cJSON *yolo_json = cJSON_Parse(buffer); 

    // Check if the parser parsed the buffer correctly
    if (yolo_json == NULL) { 

        // Error handling of the JSON parser
        const char *error_ptr = cJSON_GetErrorPtr(); 
        if (error_ptr != NULL) { 
            OS_printf("Error: %s\n", error_ptr); 
        } 

        // Deletes the strucuture if it failed
        cJSON_Delete(yolo_json); 
        return(-1) 
    } 

    return(yolo_json);
}

void headerParser(rover_array *rover, cJSON *yolo_json){
    // JSON parser grabbing the JSON header
    cJSON *header = cJSON_GetObjectItemCaseSensitive(yolo_json, "header"); 

    // Parsing the time-stamp sub-JSON
    cJSON *time_stamp = cJSON_GetObjectItemCaseSensitive(header, "stamp"); 

    // Grabbing the seconds and nano-seconds values
    cJSON *time_stamp_sec = cJSON_GetObjectItemCaseSensitive(time_stamp, "sec"); 
    cJSON *time_stamp_nanosec = cJSON_GetObjectItemCaseSensitive(time_stamp, "nanosec"); 

    // Placing the values into the rover-struct
    rover->timeStamp_sec = time_stamp_sec->valueint;
    rover->timeStamp_nanoSec = time_stamp_nanosec->valueint;

}

// Mounting the header data to the rover struct
void detectHeader(rover_state *aRover, cJSON *detection){

    // The class-ID of the YOLO object defined in the annotation process
    cJSON *class_id = cJSON_GetObjectItemCaseSensitive(detection, "class_id"); 
    aRover->rover = class_id->valueint;

    // The class name of the object in frame
    cJSON *class_name = cJSON_GetObjectItemCaseSensitive(detection, "class_name"); 
    aRover->class_name = class_name->valuestring;

    // The probability confidence score that the object observed is correctly reported
    cJSON *confidenceScore = cJSON_GetObjectItemCaseSensitive(detection, "score"); 
    aRover->confidenceScore = confidenceScore->valuedouble;

    // The tracking ID of the object
    cJSON *object_id = cJSON_GetObjectItemCaseSensitive(detection, "id"); 
    aRover->object_id = object_id->valuestring;
}

// Parses 2D-detections for YOLO-JSON
void detect2DParser(rover_state *rover, cJSON *detection){
    // Creating the internal struct for 2D bounding box
    box2D bound_box_2d;

    // Have to de-nest the JSON object
    cJSON *bbox_2D = cJSON_GetObjectItemCaseSensitive(detection, "bbox"); 

    // Grabbing the X and Y of the bounding box in the image frame
    cJSON *bbox_2D_center = cJSON_GetObjectItemCaseSensitive(bbox_2D, "center"); 
    cJSON *bbox_2D_pose = cJSON_GetObjectItemCaseSensitive(bbox_2D_center, "position"); 
    
    // Setting 2D X-coordinate
    cJSON *bbox_2D_x = cJSON_GetObjectItemCaseSensitive(bbox_2D_pose, "x"); 
    bound_box_2d->box_2d_pos_x = bbox_2D_x->valuedouble;

    // Setting 2D Y-coordinate
    cJSON *bbox_2D_y = cJSON_GetObjectItemCaseSensitive(bbox_2D_pose, "y"); 
    bound_box_2d->box_2d_pos_y = bbox_2D_y->valuedouble;

    // Setting the 2D theta
    cJSON *bbox_2D_theta = cJSON_GetObjectItemCaseSensitive(bbox_2D_center, "theta"); 
    bound_box_2d->box_2d_pos_theta = bbox_2D_theta->valuedouble;

    // Getting the size of the 2D bounding box
    // Grabbing the X and Y of the bounding box in the image frame
    cJSON *bbox_2D_sizes = cJSON_GetObjectItemCaseSensitive(bbox_2D, "size"); 

    // Setting the X-size of the bounding box
    cJSON *bbox_2D_size_x = cJSON_GetObjectItemCaseSensitive(bbox_2D_sizes, "x"); 
    bound_box_2d->box_2d_size_x = bbox_2D_size_x->valuedouble;

    // Setting the X-size of the bounding box
    cJSON *bbox_2D_size_y = cJSON_GetObjectItemCaseSensitive(bbox_2D_sizes, "y"); 
    bound_box_2d->box_2d_size_y = bbox_2D_size_y->valuedouble;

    // Adding the 2D bounding-box data to main struct
    rover->bounding_box_2d = bound_box_2d;

}

// Parses 3D-detections for YOLO-JSON
void detect3DParser(rover_state *rover, cJSON *detection){
    // Creating the internal struct for 3D bounding box
    box3D bound_box_3d;

    // Have to de-nest the JSON object
    cJSON *bbox_3D = cJSON_GetObjectItemCaseSensitive(detection, "bbox3d"); 


    //// Grabbing the X, Y and Z of the bounding box in the image frame
    cJSON *bbox_3D_center = cJSON_GetObjectItemCaseSensitive(bbox_3D, "center"); 
    cJSON *bbox_3D_position = cJSON_GetObjectItemCaseSensitive(bbox_3D_center, "position"); 
    
    // Setting 3D X-coordinate
    cJSON *bbox_3D_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "x"); 
    bound_box_3d->box_3d_pos_x = bbox_3D_x->valuedouble;

    // Setting 3D Y-coordinate
    cJSON *bbox_3D_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "y"); 
    bound_box_3d->box_3d_pos_y = bbox_3D_y->valuedouble;

    // Setting 3D Z-coordinate
    cJSON *bbox_3D_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "z"); 
    bound_box_3d->box_3d_pos_z = bbox_3D_z->valuedouble;


    //// Setting the X, Y, Z and W of the 3D orientation bounding box
    cJSON *bbox_3D_orientation = cJSON_GetObjectItemCaseSensitive(bbox_3D_center, "orientation"); 
    
    // Setting 3D X-coordinate
    cJSON *bbox_3D_Orientation_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "x"); 
    bound_box_3d->box_3d_pos_x = bbox_3D_Orientation_x->valuedouble;

    // Setting 3D Y-coordinate
    cJSON *bbox_3D_Orientation_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "y"); 
    bound_box_3d->box_3d_pos_y = bbox_3D_Orientation_y->valuedouble;

    // Setting 3D Z-coordinate
    cJSON *bbox_3D_Orientation_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "z"); 
    bound_box_3d->box_3d_pos_w = bbox_3D_Orientation_z->valuedouble;

    // Setting 3D W-coordinate
    cJSON *bbox_3D_Orientation_w = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "w"); 
    bound_box_3d->box_3d_pos_z = bbox_3D_Orientation_w->valuedouble;


    //// Getting the size of the 3D bounding box
    // Grabbing the X, Y and Z of the bounding box in the image frame
    cJSON *bbox_3D_sizes = cJSON_GetObjectItemCaseSensitive(bbox_3D, "size"); 

    // Setting the X-size of the bounding box
    cJSON *bbox_3D_size_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "x"); 
    bound_box_3d->box_3d_size_x = bbox_3D_size_x->valuedouble;

    // Setting the X-size of the bounding box
    cJSON *bbox_3D_size_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "y"); 
    bound_box_3d->box_3d_size_y = bbox_3D_size_y->valuedouble;

    // Setting the Z-size of the bounding box
    cJSON *bbox_3D_size_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "z"); 
    bound_box_3d->box_3d_size_z = bbox_3D_size_z->valuedouble;


    //// Setting the frame-ID of the bounding box
    cJSON *bbox_3D_frame_id = cJSON_GetObjectItemCaseSensitive(bbox_3D, "frame_id"); 
    bound_box_3d->box_3d_frame_id = bbox_3D_frame_id->valuestring;

    // Adding the 2D bounding-box data to main struct
    rover->bounding_box_3d = bound_box_3d;
}

void sync_fusion_injest(rover_array *rover, const char[] fileIOPath) {
//int32 sync_fusion_injest(rover_array rovers, const char[] fileIOPath) { // Future version with an array of rovers based on ID
    // Opens the YOLO-JSON file and mounts it to a cJSON linked-list
    cJSON *yolo_json = parseJSON(fileIOPath);

    // Parse out each section into Rover-Library-Struct
    // header parsing
    headerParser(&aRover, &yolo_json);

    // Parsing out the array containing 2D and 3D detections
    detections_array = cJSON_GetObjectItemCaseSensitive(yolo_json, "detections");

    // Tracks currect index of the detections array
    int pos = 0;

    // 2D-Detections parsing
    cJSON_ArrayForEach(detection, detections_array){
        // Creating a Rover struct to track the data.
        rover_state aRover;
        
        // Mounts YOLO data header to rover struct
        detectHeader(&aRover, &detection);

        // Mounts YOLO 2D bounding box data to rover struct
        detect2DParser(&aRover, &detection);

        // Mounts YOLO 3D bounding box data to rover struct
        detect3DParser(&aRover, &detection);

        // Adding the detection to the tracking rover array
        rovers_array[pos++] = aRover;
    }

    // Deleting the cJSON structure
    cJSON_Delete(yolo_json); 

    return(CFE_SUCCESS);
   
} /* End SYNC_NODE_Function */

/************************/
/*  End of File Comment */
/************************/
