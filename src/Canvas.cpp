#include "Canvas.h"
Canvas::Canvas(const std::string& map_file,double resolution)
{
    map_=cv::imread(map_file,cv::IMREAD_GRAYSCALE);
    cv::cvtColor(map_,map_,cv::COLOR_GRAY2BGR);
    resolution_=resolution;
}
void Canvas::drawPoint(const geometry::Point2d& p)
{
    int px=p.x/resolution_;
    int py=p.y/resolution_;

    cv::circle(map_,{px,py},5,{0,255,0},-1);
}
void Canvas::drawHits(const std::vector<geometry::Point2d>& hits)
{
    for(auto& p:hits)
    {
        int px=p.x/resolution_;
        int py=p.y/resolution_;

        cv::circle(map_,{px,py},2,{0,0,255},-1);
    }
}
void Canvas::show()
{
    cv::imshow("simulation",map_);
    cv::waitKey(0);
}