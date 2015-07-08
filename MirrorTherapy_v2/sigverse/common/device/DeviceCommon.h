#ifndef __DEVICE_COMMON_H__
#define __DEVICE_COMMON_H__

//Device information
#define SERVICE_NAME_KINECT_V2   "SVC_KINECT_V2"
#define SERVICE_NAME_OCULUS_DK1  "SVC_OCULUS_DK1"
#define SERVICE_NAME_OPTITRACK   "SVC_OPTITRACK"

//Device message information
// Define delimiters to split string.
#define ITEMS_DELIMITER         ";" // Delimiter to make items from whole message.
#define KEY_AND_VALUE_DELIMITER ":" // Delimiter to split to KEY and VALUE from one item.
#define VALUES_DELIMITER        "," // Delimiter to make vector(multi dimentional values) from one VALUE.

#define MSG_KEY_DEV_TYPE        "DEV_TYPE"
#define MSG_KEY_DEV_UNIQUE_ID   "DEV_UNIQUE_ID"

#define DEV_TYPE_KINECT_V2   "KINECT_V2"
#define DEV_TYPE_OCULUS_DK1  "OCULUS_DK1"
#define DEV_TYPE_OPTITRACK   "OPTITRACK"

#define DEV_UNIQUE_ID_0     "0"
#define DEV_UNIQUE_ID_1     "1"
#define DEV_UNIQUE_ID_2     "2"
#define DEV_UNIQUE_ID_3     "3"

//Parameter file information
#define PARAMETER_FILE_KEY_GENERAL_SERVICE_NAME      "General.service_name"
#define PARAMETER_FILE_KEY_GENERAL_DEVICE_TYPE       "General.device_type"
#define PARAMETER_FILE_KEY_GENERAL_DEVICE_UNIQUE_ID  "General.device_unique_id"

#endif //__DEVICE_COMMON_H__
