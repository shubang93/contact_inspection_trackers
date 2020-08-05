#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <vision_msgs/BoundingBox2D.h>
#include <string>
#include <iostream>
using namespace std;

class ButtonGUI
{
public:
  int X, Y, Width, Height;
  std::string ParamName;
  std::string Command;
  const cv::Scalar NormalColor = CV_RGB(153,255,255);
  const cv::Scalar HighlightColor = CV_RGB(153,255,204);
  cv::Scalar Color = NormalColor;
  const cv::Scalar FontColor = CV_RGB(0,0,100);

  ButtonGUI(std::string param_name = "", int x = 0, int y = 0, int width = 100, int height = 30) :
    X(x), Y(y), Width(width), Height(height), ParamName(param_name)
  {
    Command = "rosrun dynamic_reconfigure dynparam set --timeout=2 /vs_mission State " + ParamName;
  }

  bool CheckInside(int x, int y)
  {    return (x >= X && x <= X + Width && y >= Y && y <= Y + Height);  }

  void Hover(int x, int y)
  {    Color = CheckInside(x, y)? HighlightColor : NormalColor;   }

};

static const std::string MAIN_WINDOW = "Camera Feed";
static const std::string TRACKING_WINDOW = " Tracking";
cv::Rect2i roi_rect = cv::Rect2i(0, 0, 50, 50);
cv::Rect2i selected_rect;
bool roi_selected = false;
bool roi_sent = false;
cv::Mat selected_image;
cv::Size2i image_size;
cv::Size2i window_size;
  std::vector<ButtonGUI> buttons;

const int status_bar_height = 30;
const int status_bar2_height = 50;
const int separator_width = 5;
const std::string subscribed_topic = "/camera";
const std::string publish_topic = "/perception/roi/";
const std::string bbox_topic = "/perception/tracker/status";
const std::string bbox_status0 = "Please re-select ROI";
const std::string bbox_status1 = "Currently tracking";
const std::string default_status_text = "Left Click: select region  -  Mouse Wheel: change region size  -  Esc Key: Exit";
const std::string selected_status_text = "Published to " + publish_topic + "  -  Right Click: deselect region  -  " + default_status_text;
const std::string state_topic = "/vs_mission/cmd_state";
const std::string tracked_bbox_topic = "/perception/tracker/bboxImage";

std::string action_status_text = "";
std::string bbox_status_text = bbox_status0;
ros::Publisher state_pub_;

class ROISelector
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber bbox_img;
  image_transport::Publisher image_pub_;
  ros::Publisher rect_pub_;
  ros::Subscriber bbox_sub_;

public:

  static void PrintCenteredText(cv::Mat img, std::string text, int font, float size, cv::Scalar color, int x, int y, int w, int h)
  {
    int text_baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font, size, 1, &text_baseline);
    int text_x = x + (w - text_size.width) / 2;
    int text_y = y + text_size.height / 2 + h / 2;
    cv::putText(img, text, cv::Point(text_x, text_y), font, size, color, 1, 8);
  }

  static void MouseEventHandler(int event, int x, int y, int flags, __attribute__((unused)) void* userdata)
  {
    if (event == cv::EVENT_RBUTTONDOWN)
    {
      roi_selected = false;
    }
    else if (event == cv::EVENT_LBUTTONDOWN)
    {
      if (x < image_size.width && y < image_size.height)
      {
        roi_selected = true;
        roi_sent = false;
        if (x >= 0 && y >= 0)
        {
          roi_rect.x = x - roi_rect.width / 2;
          roi_rect.y = y - roi_rect.height / 2;
        }
        selected_rect = roi_rect;
      }
      else
      {
        for (int i = 0; i < (int)buttons.size(); ++i)
        {
          if (buttons[i].CheckInside(x, y))
          {
            // system((buttons[i].Command + " &").c_str());
            // ROS_WARN("GUI: Activating %s state...", buttons[i].ParamName.c_str());
            std_msgs::String msg;
            msg.data = buttons[i].ParamName;
            state_pub_.publish(msg);
            action_status_text = "Activating state: " + buttons[i].ParamName;
          }
        }
      }
    }
    else if (event == cv::EVENT_MOUSEHWHEEL)
    {
      int diff = 20;
      if (flags > 0) diff = -diff;

      if (diff > 0 || roi_rect.width > -diff) roi_rect.width += diff;
      if (diff > 0 || roi_rect.height > -diff) roi_rect.height += diff;
    }

    if (x >= 0 && y >= 0)
    {
      roi_rect.x = x - roi_rect.width / 2;
      roi_rect.y = y - roi_rect.height / 2;
    }

    for (int i = 0; i < (int)buttons.size(); ++i)
      buttons[i].Hover(x, y);
  }

  void PrintStatus(cv::Mat img_image, const char text[])
  {
    int x = 0;
    int y = window_size.height - status_bar_height - status_bar2_height;
    int w = window_size.width / 2;
    int h = status_bar2_height;
    cv::rectangle(img_image, cv::Rect2i(x + 2, y + 2, w - 4, h - 4), cv::Scalar::all(255), 1, 8);
    PrintCenteredText(img_image, bbox_status_text, cv::FONT_HERSHEY_COMPLEX, 0.8, CV_RGB(255, 255, 0), x, y, w, h);

    x = window_size.width / 2;
    cv::rectangle(img_image, cv::Rect2i(x + 2, y + 2, w - 4, h - 4), cv::Scalar::all(255), 1, 8);
    PrintCenteredText(img_image, action_status_text, cv::FONT_HERSHEY_COMPLEX, 0.5, CV_RGB(205, 102, 102), x, y, w, h);

    x = 0;
    y = window_size.height - status_bar_height;
    w = window_size.width;
    h = status_bar_height;
    cv::rectangle(img_image, cv::Rect2i(x + 2, y + 2, w - 4, h - 4), cv::Scalar::all(255), 1, 8);
    cv::putText(img_image, text, cv::Point(10, window_size.height - status_bar_height / 2 + 2), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar::all(255), 1, 8);
  }

  void bboxCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    bbox_status_text = (msg->data == 1) ? bbox_status1 : bbox_status0;


  }


  ROISelector()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish the selection
    image_sub_ = it_.subscribe(subscribed_topic, 1,
      &ROISelector::imageCb, this); //subscribes to camera feed
    image_pub_ = it_.advertise(publish_topic + "image", 1); //publishes annotated bbox image
    rect_pub_ = nh_.advertise<vision_msgs::BoundingBox2D>(publish_topic + "rect", 1); //publishes annotated bbox image
    bbox_sub_ = nh_.subscribe(bbox_topic, 10, &ROISelector::bboxCallback, this);
    bbox_img = it_.subscribe(tracked_bbox_topic,10, &ROISelector::display_tracked_bbox,this);
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic, 1);

    // Add the buttons
    buttons.push_back(ButtonGUI("Wait_hov"));
    buttons.push_back(ButtonGUI("VS_hov"));
    buttons.push_back(ButtonGUI("VS_fwd"));
    buttons.push_back(ButtonGUI("VS_bwd"));

    cv::namedWindow(MAIN_WINDOW);
    cv::setMouseCallback(MAIN_WINDOW, MouseEventHandler, NULL);

  }

  ~ROISelector()
  {
    cv::destroyWindow(MAIN_WINDOW);
  }
  void display_tracked_bbox(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr bbox;
    try
    {
      bbox = cv_bridge::toCvCopy(msg); //, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Calculate the window size
    image_size.height = bbox->image.rows;
    image_size.width = bbox->image.cols;
    cv::Mat frame(image_size.height, image_size.width, bbox->image.type(), CV_RGB(0, 0, 50));
    bbox->image.copyTo(frame(cv::Rect(0, 0, bbox->image.cols, bbox->image.rows)));
    cv::imshow(TRACKING_WINDOW, frame);

  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg);//, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Calculate the window size
    image_size.height = cv_ptr->image.rows;
    image_size.width = cv_ptr->image.cols;
    window_size.height = image_size.height + status_bar_height + status_bar2_height;
    window_size.width = image_size.width * 2 + separator_width;
    if (!buttons.empty())
      window_size.height += buttons[0].Height + 2 * separator_width;

    // Calculate the button positions and width
    if (!buttons.empty())
    {
      int buttons_width = (window_size.width - (buttons.size() + 1) * separator_width) / buttons.size();
      buttons[0].X = separator_width;
      for (int i = 0; i < (int)buttons.size(); ++i)
      {
        buttons[i].Width = buttons_width;
        if (i > 0) buttons[i].X = buttons[i-1].X + buttons[i-1].Width  + separator_width;
        buttons[i].Y = image_size.height + separator_width;
      }
    }


    // Create a new image
    cv::Mat frame(window_size.height, window_size.width, cv_ptr->image.type(), CV_RGB(0, 0, 50));

    if (roi_selected && !roi_sent)
    {
      selected_image = cv_ptr->image.clone();

      image_pub_.publish(cv_ptr->toImageMsg());
      vision_msgs::BoundingBox2D msg;
      msg.center.x = selected_rect.x + selected_rect.width / 2;
      msg.center.y = selected_rect.y + selected_rect.height / 2;
      msg.center.theta = 0;
      msg.size_x = selected_rect.width;
      msg.size_y = selected_rect.height;
      rect_pub_.publish(msg);
      roi_sent = true;
    }

    // Draw the region on the video stream
    cv::rectangle(cv_ptr->image, roi_rect, CV_RGB(255, 0, 0), 2);

    // Update GUI Window
    if (roi_selected)
    {
      cv::rectangle(selected_image, selected_rect, CV_RGB(0,255,0), 2); //draw the rectangle on the secon(selected_image)
      selected_image.copyTo(frame(cv::Rect(image_size.width + separator_width, 0, selected_image.cols, selected_image.rows))); //to display this image in the second half of the window

    }

    cv_ptr->image.copyTo(frame(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows)));

    // Draw the GUI Buttons
    for (int i = 0; i < (int)buttons.size(); ++i)
    {
      cv::rectangle(frame, cv::Rect2i(buttons[i].X, buttons[i].Y, buttons[i].Width, buttons[i].Height),
          buttons[i].Color, cv::FILLED);

      PrintCenteredText(frame, buttons[i].ParamName, cv::FONT_HERSHEY_COMPLEX, 0.5, buttons[i].FontColor,
          buttons[i].X, buttons[i].Y, buttons[i].Width, buttons[i].Height);
    }

    // Update the status
    if (roi_selected)
      PrintStatus(frame, default_status_text.c_str());
    else
      PrintStatus(frame, selected_status_text.c_str());

    cv::imshow(MAIN_WINDOW, frame);

    // Exit the program when ESC is pressed
    int key_pressed = cv::waitKey(1);
    if (key_pressed == 27) exit(0);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roi_selector_d435_gui");
  ROISelector roi_selector;
  ros::spin();
  return 0;
}
