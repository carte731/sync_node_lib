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


cJSON* parse_JSON_file(const char *fileIOPath);
//cJSON* parseJSON(const char *fileIOPath);
cJSON* ros_msg_typeCheck(rover_array *rovers, const char *data);
void headerParser(rover_array *rover, const cJSON *yolo_json);
void detect2DParser(rover_state *rover, const cJSON *detection);
void detect3DParser(rover_state *rover, const cJSON *detection);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Library Initialization Routine                                  */
/* cFE requires that a library have an initialization routine      */ 
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 SYNC_NODE_LibInit(void)
{
    
    OS_printf ("SYNC_NODE Lib Initialized.  Version %d.%d.%d.%d\n",
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
// 1.) Create the struct based on the YOLO-ROS message. - DONE
// 2.) File/IO to import JSON file - DONE
// 3.) Parse the JSON into rover-struct - DONE
// 4.) Calculate the distance from the 3D-bounding box - DONE
// 5.) Load the RGB and RGBD images into struct
// 6.) Return the struct - DONE

// Opens the JSON file and parses it
cJSON* parse_JSON_file(const char *fileIOPath){
//cJSON* parseJSON(const char *fileIOPath){

    // NOTE: Temporarily removing OSAL-fileIO for the moment, too cumbersome
    //char* test = strdup(fileIOPath);
    //const char *JSONPath = strcat(test, "YOLO-track.json");
    //int32 roverData = OS_open(JSONPath, OS_READ_ONLY, 0); 
/*
    // Opens the YOLO-JSON file 
    char catStrPath[100];
    strcpy(catStrPath, fileIOPath);
    strcat(catStrPath, "/trackOutput.json");
*/
    // Double checks if the file exist - if not, just exit
    //if(access(catStrPath, F_OK) != 0){
    if(access(fileIOPath, F_OK) != 0){
        OS_printf("File doesn't exist...\n");
        // Creates a NULL JSON object used for error checking
        cJSON *errorJSON = NULL;
        
        return(errorJSON);
    }

    FILE *roverData = fopen(fileIOPath, "r");
    //FILE *roverData = fopen(catStrPath, "r");

    // Checks if the file was loaded properly
    if(roverData == NULL){
        OS_printf("Invalid file Input...\n");
        // Creates a NULL JSON object used for error checking
        cJSON *errorJSON = NULL;
        
        //OS_close(roverData);
        fclose(roverData);
        return(errorJSON);
    }
    
    // Tells the OS to seek the end of the file
    fseek(roverData, 0L, SEEK_END);

    // Tells the program where in memory space it is.
    // This allows me to see how large the file-size is.
    int fileSize = ftell(roverData);

    // Rewind the file point back to the beginning of the file.
    rewind(roverData);
    
    // Buffer space for reading the file pointer 
    char input_buffer[fileSize]; 

    // OSAL FILE-IO operations - copies the file into buffer space
    // OSAL temporarily removed due to complexity
    //int readStatus = OS_read(roverData, input_buffer, sizeof(input_buffer));
    // Copies the file contents into buffer space
    fread(input_buffer, 1, sizeof(input_buffer), roverData);
    
    // NOTE: OSAL temporarily removed due to complexity
/*
    // Check if the file was closed correctly
    if(readStatus == OS_FS_ERROR)  {
        OS_printf("Error reading the file...");
        // Creates a NULL JSON object used for error checking
        cJSON *errorJSON = NULL;
        return(errorJSON);
    }
*/

    // NOTE: OSAL temporarily removed due to complexity
    // Uses OSAL to close the file pointer
    //int32 closeStatus = OS_close(roverData); 

    fclose(roverData);

    // NOTE: OSAL temporarily removed due to complexity
/*
    // Check if the file was closed correctly
    if(closeStatus == OS_FS_ERROR) {
        OS_printf("Error in closing the file space...");
        // Creates a NULL JSON object used for error checking
        cJSON *errorJSON = NULL;
        return(errorJSON);
    }
*/  
    // parse the JSON data into the data-structure
    cJSON *yolo_json = cJSON_Parse(input_buffer); 
    
    // Check if the parser parsed the buffer correctly
    if (yolo_json == NULL) { 

        // Error handling of the JSON parser
        const char *error_ptr = cJSON_GetErrorPtr(); 
        if (error_ptr != NULL) { 
            OS_printf("Error: %s\n", error_ptr); 
        } 

        return(yolo_json);
    } 

    return(yolo_json);
}

// Parses out the detections header portion into main part of struct 
void headerParser(rover_array *rover, const cJSON *yolo_json){
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
void detectHeader(rover_state *aRover, const cJSON *detection){

    // The class-ID of the YOLO object defined in the annotation process
    cJSON *class_id = cJSON_GetObjectItemCaseSensitive(detection, "class_id"); 
    aRover->class_id = class_id->valueint;

    // The class name of the object in frame
    cJSON *class_name = cJSON_GetObjectItemCaseSensitive(detection, "class_name"); 
    strcpy(aRover->class_name, class_name->valuestring);

    // The probability confidence score that the object observed is correctly reported
    cJSON *confidenceScore = cJSON_GetObjectItemCaseSensitive(detection, "score"); 
    aRover->confidenceScore = confidenceScore->valuedouble;

    // The tracking ID of the object
    cJSON *object_id = cJSON_GetObjectItemCaseSensitive(detection, "id"); 
    strcat(aRover->object_id, object_id->valuestring);
    
}


// Parses 2D-detections for YOLO-JSON
void detect2DParser(rover_state *rover, const cJSON *detection){
    // Creating the internal struct for 2D bounding box
    box2D bound_box_2d;

    // Have to de-nest the JSON object
    cJSON *bbox_2D = cJSON_GetObjectItemCaseSensitive(detection, "bbox"); 

    // Grabbing the X and Y of the bounding box in the image frame
    cJSON *bbox_2D_center = cJSON_GetObjectItemCaseSensitive(bbox_2D, "center"); 
    cJSON *bbox_2D_pose = cJSON_GetObjectItemCaseSensitive(bbox_2D_center, "position"); 
    
    // Setting 2D X-coordinate
    cJSON *bbox_2D_x = cJSON_GetObjectItemCaseSensitive(bbox_2D_pose, "x"); 
    bound_box_2d.box_2d_pos_x = bbox_2D_x->valuedouble;

    // Setting 2D Y-coordinate
    cJSON *bbox_2D_y = cJSON_GetObjectItemCaseSensitive(bbox_2D_pose, "y"); 
    bound_box_2d.box_2d_pos_y = bbox_2D_y->valuedouble;

    // Setting the 2D theta
    cJSON *bbox_2D_theta = cJSON_GetObjectItemCaseSensitive(bbox_2D_center, "theta"); 
    bound_box_2d.box_2d_pos_theta = bbox_2D_theta->valuedouble;

    // Getting the size of the 2D bounding box
    // Grabbing the X and Y of the bounding box in the image frame
    cJSON *bbox_2D_sizes = cJSON_GetObjectItemCaseSensitive(bbox_2D, "size"); 

    // Setting the X-size of the bounding box
    cJSON *bbox_2D_size_x = cJSON_GetObjectItemCaseSensitive(bbox_2D_sizes, "x"); 
    bound_box_2d.box_2d_size_x = bbox_2D_size_x->valuedouble;

    // Setting the X-size of the bounding box
    cJSON *bbox_2D_size_y = cJSON_GetObjectItemCaseSensitive(bbox_2D_sizes, "y"); 
    bound_box_2d.box_2d_size_y = bbox_2D_size_y->valuedouble;

    // Adding the 2D bounding-box data to main struct
    rover->bounding_box_2d = bound_box_2d;

}

// Parses 3D-detections for YOLO-JSON
void detect3DParser(rover_state *rover, const cJSON *detection){
    // Creating the internal struct for 3D bounding box
    box3D bound_box_3d;

    // Have to de-nest the JSON object
    cJSON *bbox_3D = cJSON_GetObjectItemCaseSensitive(detection, "bbox3d");

    //// Grabbing the X, Y and Z of the bounding box in the image frame
    cJSON *bbox_3D_center = cJSON_GetObjectItemCaseSensitive(bbox_3D, "center"); 
    cJSON *bbox_3D_position = cJSON_GetObjectItemCaseSensitive(bbox_3D_center, "position"); 

    // Setting 3D X-coordinate
    cJSON *bbox_3D_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "x"); 
    bound_box_3d.box_3d_pos_x = bbox_3D_x->valuedouble;

    // Setting 3D Y-coordinate
    cJSON *bbox_3D_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "y"); 
    bound_box_3d.box_3d_pos_y = bbox_3D_y->valuedouble;

    // Setting 3D Z-coordinate
    cJSON *bbox_3D_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_position, "z"); 
    bound_box_3d.box_3d_pos_z = bbox_3D_z->valuedouble;


    //// Setting the X, Y, Z and W of the 3D orientation bounding box
    cJSON *bbox_3D_orientation = cJSON_GetObjectItemCaseSensitive(bbox_3D_center, "orientation"); 
    
    // Setting 3D X-coordinate
    cJSON *bbox_3D_Orientation_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "x"); 
    bound_box_3d.box_3d_orient_x = bbox_3D_Orientation_x->valuedouble;

    // Setting 3D Y-coordinate
    cJSON *bbox_3D_Orientation_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "y"); 
    bound_box_3d.box_3d_orient_y = bbox_3D_Orientation_y->valuedouble;

    // Setting 3D Z-coordinate
    cJSON *bbox_3D_Orientation_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "z"); 
    bound_box_3d.box_3d_orient_z = bbox_3D_Orientation_z->valuedouble;

    // Setting 3D W-coordinate
    cJSON *bbox_3D_Orientation_w = cJSON_GetObjectItemCaseSensitive(bbox_3D_orientation, "w"); 
    bound_box_3d.box_3d_orient_w = bbox_3D_Orientation_w->valuedouble;


    //// Getting the size of the 3D bounding box
    // Grabbing the X, Y and Z of the bounding box in the image frame
    cJSON *bbox_3D_sizes = cJSON_GetObjectItemCaseSensitive(bbox_3D, "size"); 

    // Setting the X-size of the bounding box
    cJSON *bbox_3D_size_x = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "x"); 
    bound_box_3d.box_3d_size_x = bbox_3D_size_x->valuedouble;

    // Setting the X-size of the bounding box
    cJSON *bbox_3D_size_y = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "y"); 
    bound_box_3d.box_3d_size_y = bbox_3D_size_y->valuedouble;

    // Setting the Z-size of the bounding box
    cJSON *bbox_3D_size_z = cJSON_GetObjectItemCaseSensitive(bbox_3D_sizes, "z"); 
    bound_box_3d.box_3d_size_z = bbox_3D_size_z->valuedouble;


    //// Setting the frame-ID of the bounding box
    cJSON *bbox_3D_frame_id = cJSON_GetObjectItemCaseSensitive(bbox_3D, "frame_id"); 
    //bound_box_3d.box_3d_frame_id = strdup(bbox_3D_frame_id->valuestring);
    strcat(bound_box_3d.box_3d_frame_id, bbox_3D_frame_id->valuestring);
 

    // Adding the 2D bounding-box data to main struct
    rover->bounding_box_3d = bound_box_3d;
}

void detect2DKeypoint(rover_state *rover, const cJSON *detection){

    keypoint2D  keypoint_2D_Arr[MAX_KEYPOINT_ARRAY_SIZE];

    cJSON *keypoint_2D = cJSON_GetObjectItemCaseSensitive(detection, "keypoints");
    cJSON *keypoint_2D_array = cJSON_GetObjectItemCaseSensitive(keypoint_2D, "data");
    cJSON *keypoint_element = NULL;
    cJSON *tempJSON = NULL;

    int itr = 0;
    cJSON_ArrayForEach(keypoint_element, keypoint_2D_array){

        keypoint2D  keypoint_2D_Ele;

        // Retrieving the Keypoint ID from JSON object
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_element, "id");
        keypoint_2D_Ele.keypoint_id = tempJSON->string;

        // Retrieving the Keypoint confidence-score from JSON object
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_element, "score");
        keypoint_2D_Ele.confidence_score = tempJSON->valuedouble;
        
        // Retrieving the x & Y coordinates from JSON object
        cJSON *keypoint_2D_xy = cJSON_GetObjectItemCaseSensitive(keypoint_element, "point");
        // X-COOR
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_2D_xy, "x");
        keypoint_2D_Ele.point_x = tempJSON->valuedouble;
        // Y-COOR
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_2D_xy, "y");
        keypoint_2D_Ele.point_y = tempJSON->valuedouble;

        // Adding the new keypoint element to the 
        rover->keypoint_2D_listing[itr++] = keypoint_2D_Ele;
    }

}

void detect3DKeypoint(rover_state *rover, const cJSON *detection){


    keypoint3D  keypoint_3D_Arr[MAX_KEYPOINT_ARRAY_SIZE];

    cJSON *keypoint_3D = cJSON_GetObjectItemCaseSensitive(detection, "keypoints3d");
    cJSON *keypoint_3D_array = cJSON_GetObjectItemCaseSensitive(keypoint_3D, "data");
    cJSON *keypoint_element = NULL;
    cJSON *tempJSON = NULL;

    int itr = 0;
    cJSON_ArrayForEach(keypoint_element, keypoint_3D_array){

        keypoint3D  keypoint_3D_Ele;

        // Retrieving the Keypoint ID from JSON object
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_element, "id");
        keypoint_3D_Ele.keypoint_id = tempJSON->string;

        // Retrieving the Keypoint confidence-score from JSON object
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_element, "score");
        keypoint_3D_Ele.confidence_score = tempJSON->valuedouble;
        
        // Retrieving the X, Y & Z coordinates from JSON object
        cJSON *keypoint_3D_xy = cJSON_GetObjectItemCaseSensitive(keypoint_element, "point");
        // X-COOR
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_3D_xy, "x");
        keypoint_3D_Ele.point_x = tempJSON->valuedouble;
        // Y-COOR
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_3D_xy, "y");
        keypoint_3D_Ele.point_y = tempJSON->valuedouble;
        // Z-COOR
        tempJSON = cJSON_GetObjectItemCaseSensitive(keypoint_3D_xy, "z");
        keypoint_3D_Ele.point_z = tempJSON->valuedouble;
        
        // Adding the new keypoint element to the 
        rover->keypoint_3D_listing[itr++] = keypoint_3D_Ele;
    }
}


// Main library function that parses YOLO-JSON file into Rover structs
int32 ros_msg_parse(rover_array *rovers, const char *data) {

    cJSON *yolo_json = ros_msg_typeCheck(rovers, data);

    if(yolo_json == NULL){
        // Deletes the strucuture if it failed
        cJSON_Delete(yolo_json);
        return(-1);
    }

    // Parse out each section into Rover-Library-Struct

    // header parsing
    headerParser(rovers, yolo_json);

    // Creating NULL cJSON objects for iterating over
    const cJSON *detection = NULL;
    const cJSON *detections_array = NULL;

    // Parsing out the array containing 2D and 3D detections
    detections_array = cJSON_GetObjectItemCaseSensitive(yolo_json, "detections");

    // If there are no detections, don't do anything
    if(cJSON_GetArraySize(detections_array) == 0){
        cJSON_Delete(yolo_json); 
        return(-1);
    }

    // Tracks currect index of the detections array
    int pos = 0;

    // 2D-Detections parsing
    cJSON_ArrayForEach(detection, detections_array){
        // Creating a Rover struct to track the data.
        rover_state aRover;

        // Mounts YOLO data header to rover struct
        detectHeader(&aRover, detection);

        // Mounts YOLO 2D bounding box data to rover struct
        detect2DParser(&aRover, detection);

        // Mounts YOLO 3D bounding box data to rover struct
        detect3DParser(&aRover, detection);

        // Mounts 2D-keypoint data to rover struct
        detect2DKeypoint(&aRover, detection);

        // Mounts 3D-keypoint data to rover struct
        detect3DKeypoint(&aRover, detection);

        // Calculating the square for each 3D coordinate for distance calculation
        double xCoor = pow(aRover.bounding_box_3d.box_3d_pos_x, 2);
        double yCoor = pow(aRover.bounding_box_3d.box_3d_pos_y, 2);
        double zCoor = pow(aRover.bounding_box_3d.box_3d_pos_z, 2);
 
        // Calculating the distance using euclidean space
        aRover.distance = sqrt(xCoor + yCoor + zCoor);

        // Adding the detection to the tracking rover array
        rovers->rovers_array[pos++] = aRover;

        // Tracking how many positions in the array are actually used
        rovers->arrayLen++;
    }

    // Deleting the cJSON structure
    cJSON_Delete(yolo_json); 

    return(CFE_SUCCESS);
   
}

cJSON* ros_msg_typeCheck(rover_array *rovers, const char *data){

    // Used for checking if string parameter is a JSON string
    cJSON *jsonCheck = NULL;

    // Creates Yolo-JSON file string for checking
    int dataLen = strlen(data) + 20;
    char pathCheck[dataLen];
    strcpy(pathCheck, data);
    strcat(pathCheck, "/trackOutput.json");
    struct stat fileCheck;
    
    // Checks if the file exist - if so, mount and return JSON
    if(stat(pathCheck, &fileCheck) == 0){
        
        if(S_ISREG(fileCheck.st_mode)){
            jsonCheck = parse_JSON_file(pathCheck);
        
            return(jsonCheck);
        }

    } 
    
    // If not a file, check if the string is a JSON string
    jsonCheck = cJSON_Parse(data);

    //char *string = cJSON_Print(jsonCheck);
    //if (string == NULL) {
    if (jsonCheck == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr(); 
        if (error_ptr != NULL) { 
            OS_printf("Error: %s\n", error_ptr); 
        } 

        return(NULL);

    } 
    
    return(jsonCheck);
   

}

    /* End SYNC_NODE_Function */

/************************/
/*  End of File Comment */
/************************/
