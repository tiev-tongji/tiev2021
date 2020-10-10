#ifndef DGC_APPLANIX_HISTORY_H
#define DGC_APPLANIX_HISTORY_H

//#include <applanix_messages.h>

#define APPLANIX_HISTORY_LENGTH     200
typedef struct {
    double smooth_x;     /** Velocity-integrated, smooth position (East).  0=where applanix was started */
    double smooth_y;     /** Velocity-integrated, smooth position (North). 0=where applanix was started */
    double smooth_z;     /** Velocity-integrated, smooth position (Up).    0=where applanix was started */
    double latitude;     /**< Latitude, in degrees */
    double longitude;    /**< Longitude, in degrees */
    double altitude;     /**< Altitude, in meters */
    float v_north;       /**< north velocity, in m/s */
    float v_east;        /**< east velocity, in m/s */
    float v_up;          /**< up velocity, in m/s */
    float speed;         /**< Magitude of velocity vector, in m/s */
    float track;         /**< planar heading of velocity vector, in radians */
    double roll;         /**< roll, in radians */
    double pitch;        /**< pitch, in radians */
    double yaw;          /**< yaw, in radians */
    double ar_roll;      /**< roll rate, in rad/sec */
    double ar_pitch;     /**< pitch rate, in rad/sec */
    double ar_yaw;       /**< yaw rate, in rad/sec */
    double a_x;          /**< acceleration in x, in m/s/s (body frame) */
    double a_y;          /**< acceleration in y, in m/s/s (body frame) */
    double a_z;          /**< acceleration in z, in m/s/s (body frame) */
    double wander;       /**< wander angle, in radians */
    unsigned int ID;     /**< unique ID for internal tracking */
    int postprocess_code;  /**< 0 = Real Time.  1 = Post Processed.  2 = Post Processed with Base Station. */
    double hardware_timestamp;    /**< Timestamp from the Applanix hardware, in UTC seconds of the week. */
    int hardware_time_mode;        /**< Mode of timestamp from the Applanix hardware.  0 = None.  1 = Acquire.  2 = Locked. */
    double timestamp;    /**< DGC timestamp */
    char host[10];       /**< hostname associated with timestamp */
} ApplanixPose;

  
typedef struct applanix_elem_t {

  applanix_elem_t                * next;
  applanix_elem_t                * prev;
  ApplanixPose                     data;

} *applanix_elem_p;

typedef struct {

  applanix_elem_p                  current;

} applanix_history_t, *applanix_history_p;


 
#endif
