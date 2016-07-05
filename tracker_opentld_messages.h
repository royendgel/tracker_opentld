/*********************************************************
	 ---   tracker_opentld messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef TRACKER_OPENTLD_MESSAGES_H
#define TRACKER_OPENTLD_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Message Struct */
typedef struct {
	int box_x;	                    /*the x position of the current boundbox*/
	int box_y;	                    /*the x position of the current boundbox*/
	int box_width;	              	/*the width of the current boundbox*/
	int box_height;                 /*the height of the current boundbox*/
	double confidence;            	/*current confidence of the tracking*/
	int camera_side;      	/*side of the camera used (0-left,  1-right)*/
	int camera_id;					/*current camera_id used*/
	int camera_width;        		/**<The x dimension of the image in pixels. */
	int camera_height;      		/**<The y dimension of the image in pixels. */
	double timestamp; 				/* !!! obrigatory !!! */
	char *host; 					/* !!! obrigatory !!! */

} carmen_tracker_opentld_message;

/* The message's name, will be used for message registration in IPC Central module */
#define      CARMEN_TRACKER_OPENTLD_MESSAGE_NAME       "carmen_tracker_opentld_message"

/* The message's format, will be used for message data marshalling (serialization) for network socket transport. */
#define      CARMEN_TRACKER_OPENTLD_MESSAGE_FMT        "{int,int,int,int,double,int,int,int,int,double,string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
