/*
/*
/* Based on work available at: https://gist.github.com/anujonthemove/7b35b7c1e05f01dd11d74d94784c1e58
/* by inaciose
/*
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <list>
#include <opencv2/saliency/saliencySpecializedClasses.hpp>

#define PI 3.1415926

int frameWidth = 640;
int frameHeight = 480;

int alpha_ = 90, beta_ = 90, gamma_ = 90;
int f_ = 500, dist_ = 500;

CV_EXPORTS_W void get_regions_of_interest(cv::InputArray _src, cv::OutputArrayOfArrays mv, cv::OutputArrayOfArrays mv2 = cv::noArray());

#define _DEBUG

void get_regions_of_interest(cv::InputArray _src, cv::OutputArrayOfArrays _rois, cv::OutputArrayOfArrays _contours)
{

    // Check that the first argument is an image and the second a vector of images.
    CV_Assert(_src.isMat() && !_src.depth() && (_src.channels() == 1 || _src.channels() == 3) && _rois.isMatVector() && (!_contours.needed() || (_contours.needed() && _contours.isMatVector()) ) );

    static cv::Ptr<cv::saliency::StaticSaliencySpectralResidual> saliency;

    if(!saliency)
        saliency = cv::saliency::StaticSaliencySpectralResidual::create();

    cv::Mat src = _src.getMat();
    cv::Mat gray;

    if(src.depth() == src.type())
        gray = src;
    else
        cv::cvtColor(src,gray,cv::COLOR_BGR2GRAY);

    bool is_ctr_needed = _contours.needed();
    std::list<cv::Mat> final_ctrs;

    // Step 1) Process the saliency in order to segment the dices.

    cv::Mat saliency_map;
    cv::Mat binary_map;

    saliency->computeSaliency(src,saliency_map);
    saliency->computeBinaryMap(saliency_map,binary_map);


    saliency_map.release();

    // Step 2) From the binary map get the regions of interest.

    cv::Mat1i stats;
    std::vector<cv::Mat> rois;

    cv::Mat labels;
    cv::Mat centroids;

    cv::connectedComponentsWithStats(binary_map, labels, stats, centroids);


    labels.release();
    centroids.release();

    // prepare the memory
    rois.reserve(stats.rows-1);

    // Sort the stats in order to remove the background.

    stats = stats.colRange(0,stats.cols-1);

    // Extract the rois.

    for(int i=0;i<stats.rows;i++)
    {
        cv::Rect roi = *reinterpret_cast<cv::Rect*>(stats.ptr<int>(i));

        if(static_cast<std::size_t>(roi.area()) == gray.total())
            continue;

        rois.push_back(gray(roi));
#ifdef _DEBUG
        cv::imshow("roi_"+std::to_string(i),gray(roi));
#endif
    }



    // Step 3) Refine.

    // Because the final number of shape cannot be determine in advance it is better to use a linked list than a vector.
    // In practice except if there is a huge number of elements to work with the performance will be almost the same.
    std::list<cv::Mat> shapes;

    int cnt=0;
    for(const cv::Mat& roi : rois)
    {

        cv::Mat tmp = roi.clone();

        // Slightly sharpen the regions contours
        cv::morphologyEx(tmp,tmp, cv::MORPH_CLOSE, cv::noArray());
        // Reduce the influence of local unhomogeneous illumination.
        cv::GaussianBlur(tmp,tmp,cv::Size(31,31), 5);

        cv::Mat thresh;
        // Binarize the image.
        cv::threshold(roi,thresh,0.,255.,cv::THRESH_BINARY | cv::THRESH_OTSU);
#ifdef _DEBUG
        cv::imshow("thresh"+std::to_string(cnt++),thresh);
#endif
        // Find the contours of each sub region on interest
        std::vector<cv::Mat> contours;

        cv::findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat dc;

        cv::merge(std::vector<cv::Mat>(3,thresh),dc);

//        cv::drawContours(dc, contours,-1,cv::Scalar(0.,0.,255),2);
//        cv::imshow("ctrs"+std::to_string(cnt),dc);

        // Extract the sub-regions

        if(is_ctr_needed)
        {
            for(const cv::Mat& ctrs: contours)
            {

                cv::Rect croi = cv::boundingRect(ctrs);

                // If the sub region is to big or to small it is depreate
                if(static_cast<std::size_t>(croi.area()) == roi.total() || croi.area()<50)
                    continue;

                final_ctrs.push_back(ctrs);

                shapes.push_back(roi(croi));

#ifdef _DEBUG
                cv::rectangle(dc,croi,cv::Scalar(0.,0.,255.));

                cv::imshow("sub_roi_"+std::to_string(cnt++),roi(croi));
#endif
            }
        }
        else
        {
            for(const cv::Mat& ctrs: contours)
            {

                cv::Rect croi = cv::boundingRect(ctrs);

                // If the sub region is to big or to small it is depreate
                if(static_cast<std::size_t>(croi.area()) == roi.total() || croi.area()<50)
                    continue;

                shapes.push_back(roi(croi));

#ifdef _DEBUG
                cv::rectangle(dc,croi,cv::Scalar(0.,0.,255.));

                cv::imshow("sub_roi_"+std::to_string(cnt++),roi(croi));
#endif

            }
        }

    }
#ifdef _DEBUG
    cv::waitKey(-1);
#endif

    // Final Step: set the output

    _rois.create(shapes.size(),1,CV_8U);
    _rois.assign(std::vector<cv::Mat>(shapes.begin(),shapes.end()));

    if(is_ctr_needed)
    {
        _contours.create(final_ctrs.size(),1,CV_32SC2);
        _contours.assign(std::vector<cv::Mat>(final_ctrs.begin(), final_ctrs.end()));
    }

}




void imageSubCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat imgsrc;
  cv::Mat imgdst;
  try
  {
    imgsrc = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  cv::resize(imgsrc, imgsrc, cv::Size(frameWidth, frameHeight));

  double focalLength, dist, alpha, beta, gamma; 

  alpha =((double)alpha_ -90) * PI/180;
  beta =((double)beta_ -90) * PI/180;
  gamma =((double)gamma_ -90) * PI/180;
  focalLength = (double)f_;
  dist = (double)dist_;

  cv::Size image_size = imgsrc.size();
  double w = (double)image_size.width, h = (double)image_size.height;

  // Projecion matrix 2D -> 3D
  cv::Mat A1 = (cv::Mat_<float>(4, 3)<< 
    1, 0, -w/2,
    0, 1, -h/2,
    0, 0, 0,
    0, 0, 1 );

  // Rotation matrices Rx, Ry, Rz
  cv::Mat RX = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, 0,
    0, cos(alpha), -sin(alpha), 0,
    0, sin(alpha), cos(alpha), 0,
    0, 0, 0, 1 );

  cv::Mat RY = (cv::Mat_<float>(4, 4) << 
    cos(beta), 0, -sin(beta), 0,
    0, 1, 0, 0,
    sin(beta), 0, cos(beta), 0,
    0, 0, 0, 1	);

  cv::Mat RZ = (cv::Mat_<float>(4, 4) << 
    cos(gamma), -sin(gamma), 0, 0,
    sin(gamma), cos(gamma), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1	);

  // R - rotation matrix
  cv::Mat R = RX * RY * RZ;

  // T - translation matrix
  cv::Mat T = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, 0,  
    0, 1, 0, 0,  
    0, 0, 1, dist,  
    0, 0, 0, 1); 
  
  // K - intrinsic matrix 
  cv::Mat K = (cv::Mat_<float>(3, 4) << 
    focalLength, 0, w/2, 0,
    0, focalLength, h/2, 0,
    0, 0, 1, 0
    ); 

  cv::Mat transformationMat = K * (T * (R * A1));

  cv::warpPerspective(imgsrc, imgdst, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

  // shape detection
  std::vector<cv::Mat> rois;

  get_regions_of_interest(imgsrc,rois);


  cv::imshow("Source", imgsrc);
  cv::imshow("Result", imgdst);
  cv::waitKey(30);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perspective_explorer");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string sub_topic;
  //pnh.param<std::string>("sub_topic", sub_topic, "usb_cam/image_raw"); 
  pnh.param<std::string>("sub_topic", sub_topic, "/ackermann_vehicle/camera/rgb/image_raw"); 

  pnh.param<int>("width", frameWidth, 640); 
  pnh.param<int>("height", frameHeight, 480); 

  cv::namedWindow("Result", 1);

	cv::createTrackbar("Pitch", "Result", &alpha_, 180);
	cv::createTrackbar("Roll", "Result", &beta_, 180);
	cv::createTrackbar("Yaw", "Result", &gamma_, 180);
	cv::createTrackbar("f", "Result", &f_, 2000);
	cv::createTrackbar("Distance", "Result", &dist_, 2000);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(sub_topic, 1, imageSubCallback);
  ros::spin();

  cv::destroyWindow("Result");
}