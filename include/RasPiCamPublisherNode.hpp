#define VCOS_ALWAYS_WANT_LOGGING

#define VERSION_STRING "v1.2"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "RaspiCLI.h"
#include "RaspiCamControl.h"

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

class RasPiCamPublisher : public rclcpp::Node {
public:
    //COMPOSITION_PUBLIC

    RasPiCamPublisher();

    ~RasPiCamPublisher();

private:
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_img;
    
    static const int IMG_BUFFER_SIZE = 10 * 1024 * 1024;
    
    /** Structure containing all state information for the current run
    */
    typedef struct {
        int isInit;
        int width;      /// Requested width of image
        int height;     /// requested height of image
        int framerate;  /// Requested frame rate (fps)
        int quality;
    
        RASPICAM_CAMERA_PARAMETERS
        camera_parameters;  /// Camera setup parameters
    
        MMAL_COMPONENT_T *camera_component;     /// Pointer to the camera component
        MMAL_COMPONENT_T *encoder_component;    /// Pointer to the encoder component
        MMAL_CONNECTION_T *preview_connection;  /// Pointer to the connection
                                                /// from camera to preview
        MMAL_CONNECTION_T *encoder_connection;  /// Pointer to the connection
                                                /// from camera to encoder
    
        MMAL_POOL_T *video_pool;    /// Pointer to the pool of buffers used by
                                    /// encoder output port
        MMAL_POOL_T *encoder_pool;  /// Pointer to the pool of buffers used by
                                    /// encoder output port
    } RASPIVID_STATE;
    
    RASPIVID_STATE state_srv;
    int skip_frames = 0;
    int frames_skipped = 0;

    /** Struct used to pass information in encoder port userdata to callback
    */
    typedef struct {
        unsigned char *buffer[2];  /// File handle to write buffer data to.
        RASPIVID_STATE *pstate;    /// pointer to our state in case required in callback
        int abort;  /// Set to 1 in callback if an error occurs to attempt to abort the capture
        int frame;
        int id;
        RasPiCamPublisher *pThis;  // pointer to own class instance
    } PORT_USERDATA;

    /**
   *  buffer header callback function for encoder
   *
   *  Callback will dump buffer data to the specific file
   *
   * @param port Pointer to port from which callback originated
   * @param buffer mmal buffer header pointer
   */
    static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

    /**
   * Assign a default set of parameters to the state passed in
   *
   * @param state Pointer to state structure to assign defaults to
   */
    void get_status(RASPIVID_STATE *state);

    /**
   * Create the camera component, set up its ports
   *
   * @param state Pointer to state control struct
   *
   * @return 0 if failed, pointer to component if successful
   *
   */
    static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state);

    /**
   * Destroy the camera component
   *
   * @param state Pointer to state control struct
   *
   */
    static void destroy_camera_component(RASPIVID_STATE *state);

    /**
    * Create the encoder component, set up its ports
    *
    * @param state Pointer to state control struct
    *
    * @return MMAL_SUCCESS if all OK, something else otherwise
    *
    */
    static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state);

    /**
   * Destroy the encoder component
   *
   * @param state Pointer to state control struct
   *
   */
    static void destroy_encoder_component(RASPIVID_STATE *state);

    /**
   * Connect two specific ports together
   *
   * @param output_port Pointer the output port
   * @param input_port Pointer the input port
   * @param Pointer to a mmal connection pointer, reassigned if function
   * successful
   * @return Returns a MMAL_STATUS_T giving result of operation
   *
   */
    static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection);

    /**
   * Checks if specified port is valid and enabled, then disables it
   *
   * @param port  Pointer the port
   *
   */
    static void check_disable_port(MMAL_PORT_T *port);

    /**
   * init_cam

   */
    int init_cam(RASPIVID_STATE *state);

    int start_capture(RASPIVID_STATE *state);

    int close_cam(RASPIVID_STATE *state);

};  // node