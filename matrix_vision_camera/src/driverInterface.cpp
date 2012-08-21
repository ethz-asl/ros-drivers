// simple node which interacts with the camera settings

#include "ros/ros.h"
#include <cstdlib>
#include <string>
#include "cv.h"
#include "matrix_vision_camera/propertyMap.h"
#include "nodes/propertyMapInterface.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>

using namespace std;
//usign namespace cv;
namespace enc = sensor_msgs::image_encodings;


bool saveImageToFileBool = 1;
bool continuousCapture = 0;

int sendAndPrintResult(ros::ServiceClient client, matrix_vision_camera::propertyMap srv) {

    if (client.call(srv)) {
        if (srv.response.status == 0) {
            cout << "An ERROR occured:" << endl;
        }
        cout << srv.response.result << endl;
        return srv.response.status;
        
    }
    else {
        ROS_ERROR("Failed to call service!");
        cout << "Failed to call service!";
        return 0;
    }
    
}

string getInputForQuestion(string question) {
    cout << question << endl;
    string out;
    cin >> out;
    return out;
}



void saveImageToFile(const sensor_msgs::ImageConstPtr& msg, string filename) {

    // convert the mesasge to cv image pointer
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imwrite(filename, cv_ptr->image);

}



void imageMessageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (saveImageToFileBool && !continuousCapture) {
        
        std::time_t t = std::time(0); 
        std::stringstream out;
        out << t;
        saveImageToFile(msg, "images/im_" + out.str() + ".jpg" );
        cout << "Received and saved Image to: " << "images/im_" + out.str() + ".jpg";
        
    }
}





int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "driverInterface");
    
    
    cout << "Initialising ROS and service client side...\n";
    
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<matrix_vision_camera::propertyMap>("/camera/pollPropertyList");
    // create a listener
    ros::Subscriber sub = n.subscribe("/camera/image_raw", 1000, imageMessageCallback);
    // asynchronous spinning to save the images in the background
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    cout << "Done.\n";
    
    string cmd;
    
    // the main loop:    
    bool boRun = true;
    while( boRun ) {
        
        
        matrix_vision_camera::propertyMap srv;
        // clean request:
        srv.request.command = 0;
        srv.request.identifier = "";
        srv.request.value = "";
        
        // read command:
        cin >> cmd;
        
        
        
        // check if service still exists:
        if( ! client.exists() ) {
            cout << "Service currently unavailable!" << endl;
            continue;
        }
        
    
        // chose command.
        /// LIST
        if ( cmd == "list" || cmd == "l") {
            // request setup:
            srv.request.command = PROPERTYMAP_GET_PROPERTY_LIST;
            sendAndPrintResult(client, srv);
        }
        /// FIND
        else if ( cmd == "find" || cmd == "f") {
            srv.request.command = PROPERTYMAP_SEARCH_PROPERTY_MAP;
            srv.request.identifier = getInputForQuestion("Enter Search String:");
            
            sendAndPrintResult(client, srv);
            
        }
        /// SET
        else if ( cmd == "set" || cmd == "s") {
            
            srv.request.identifier = getInputForQuestion("Enter Property Identifier:");

            // send a request to the server to get the info:
            srv.request.command = PROPERTYMAP_GET_PROPERTY_INFO;
            int status = sendAndPrintResult(client, srv);
            
            if (status == 0) {
                cout << "This property does not exist!";
            }
            else {
                // now continue and ask for the value:
                srv.request.command = PROPERTYMAP_SET_PROPERTY;
                srv.request.value = getInputForQuestion("Enter new Property Value:");
                // send parameter set request:
                sendAndPrintResult(client, srv);
            }
            
        }
        /// INFO
        else if ( cmd == "info" || cmd == "i" ) {
            srv.request.command = PROPERTYMAP_GET_PROPERTY_INFO;
            srv.request.identifier = getInputForQuestion("Enter Property Identifier:");
            sendAndPrintResult(client, srv);
        }
        /// SAVE
        else if ( cmd == "save" ) {
            srv.request.command = PROPERTYMAP_LOAD_SETTINGS;
            srv.request.identifier = getInputForQuestion("File Path:");
            sendAndPrintResult(client, srv);
        }
        /// LOAD
        else if( cmd == "load" ) {
            srv.request.command = PROPERTYMAP_SAVE_SETTINGS;
            srv.request.identifier = getInputForQuestion("File Path:");
            sendAndPrintResult(client, srv);
        }
        else if( cmd == "capture" || cmd == "c") {
            srv.request.command = DI_START_CAPTURE_PROCESS;
            sendAndPrintResult(client, srv);
            continuousCapture = 1;
        }
        else if( cmd == "stop" || cmd == "s") {
            srv.request.command = DI_STOP_CAPTURE_PROCESS;
            sendAndPrintResult(client, srv);
            continuousCapture = 0;
        }
        else if( cmd == "single" ) {
            srv.request.command = DI_CAPTURE_SINGLE_FRAME;
            sendAndPrintResult(client, srv);
        }
        else if( cmd == "restart" ) {
            srv.request.command = DI_RESTART_DEVICE;
            sendAndPrintResult(client, srv);
        }
        else if( cmd == "close" ) {
            srv.request.command = DI_CLOSE_DEVICE;
            sendAndPrintResult(client, srv);
        }
        else if( cmd == "open" ) {
            srv.request.command = DI_OPEN_DEVICE;
            sendAndPrintResult(client, srv);
        }
        /// QUIT
        else if ( cmd == "quit" || cmd == "q") {
            boRun = false;
            continue;
        }
        /// HELP
        else {
            cout << "Commands are: \n";
            cout << "\t quit: exit the program \n";
            cout << "\t list: list all available properties \n";
            cout << "\t set: set a property \n";
            cout << "\t info: get property details \n";
            cout << "\t save: save camera configuration \n";
            cout << "\t load: load camera configuration \n";
            cout << "\t capture: start continuous capture\n";
            cout << "\t stop: stop continuous capture \n";
            cout << "\t single: get a single image \n";
            cout << "\t restart: close and reopen device \n";
            cout << "\t close: close device \n";
            cout << "\t open: open device \n";
        }
    
        
        cout << "\n------\n";
    
    }


    
    
    

    
    return 0;
}
