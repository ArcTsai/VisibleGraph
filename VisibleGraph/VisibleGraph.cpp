// VisibleGraph.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <boost/config.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/polygon/segment_data.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/voronoi_builder.hpp>
#include <boost/polygon/voronoi_diagram.hpp>
#include <boost/polygon/voronoi_geometry_type.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost\graph\astar_search.hpp>
#include <boost\graph\dijkstra_shortest_paths.hpp>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <time.h>

struct Point {
  int a;
  int b;
  Point(int x, int y) : a(x), b(y) {}
};

struct Segment {
  Point p0;
  Point p1;
  Segment(int x1, int y1, int x2, int y2) : p0(x1, y1), p1(x2, y2) {}
};

namespace boost {
namespace polygon {

template <> struct geometry_concept<Point> { typedef point_concept type; };

template <> struct point_traits<Point> {
  typedef int coordinate_type;

  static inline coordinate_type get(const Point &point, orientation_2d orient) {
    return (orient == HORIZONTAL) ? point.a : point.b;
  }
};

template <> struct geometry_concept<Segment> { typedef segment_concept type; };

template <> struct segment_traits<Segment> {
  typedef int coordinate_type;
  typedef Point point_type;

  static inline point_type get(const Segment &segment, direction_1d dir) {
    return dir.to_int() ? segment.p1 : segment.p0;
  }
};
} // namespace polygon
} // namespace boost

bool ContoursSort(std::vector<cv::Point> contour1,
                  std::vector<cv::Point> contour2) {
  return (cv::contourArea(contour1) > cv::contourArea(contour2));
}

int main() {
  long begin_time = clock();
  cv::Mat Image = cv::imread(
      "D:\\Projects\\GProject\\Data\\ISPRS\\DispMat\\DP14,2.3,LR.tif", 0);

  // cv::threshold(Image, Image, 30, 255, cv::THRESH_BINARY);
  cv::adaptiveThreshold(Image, Image, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY_INV, 101, 0);
  std::vector<std::vector<cv::Point>> contours;
  /*cv::findContours(Image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  contours.erase(std::remove_if(contours.begin(), contours.end(),
                                [](const std::vector<cv::Point> &c) {
                                  return cv::contourArea(c) < 150;
                                }),
                 contours.end());

  Image.setTo(0);
  cv::drawContours(Image, contours, -1, cv::Scalar(255), cv::FILLED);
  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(Image, Image, cv::MORPH_CLOSE, element);*/
  // cv::threshold(Image, Image, 10, 255, cv::THRESH_BINARY_INV);

  std::vector<std::vector<cv::Point>> contours1;
  cv::findContours(Image, contours1, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
  std::vector<std::vector<cv::Point>> contours_poly(contours1.size());

  cv::Mat Mask = cv::Mat::zeros(Image.size(), CV_8UC1);
  for (int i = 0; i < contours1.size(); i++) {
    if (contours1[i].size() > 10) {
      cv::approxPolyDP(cv::Mat(contours1[i]), contours_poly[i], 3, true);
    } else {
      contours_poly[i] = contours1[i];
    }
  }

  contours_poly.erase(std::remove_if(contours_poly.begin(), contours_poly.end(),
                                     [](const std::vector<cv::Point> &c) {
                                       return cv::contourArea(c) < 100;
                                     }),
                      contours_poly.end());

  cv::drawContours(Mask, contours_poly, -1, cv::Scalar(255), 1, cv::LINE_8);
  Image.setTo(0);
  cv::drawContours(Image, contours_poly, -1, cv::Scalar(255), cv::FILLED);

  std::vector<Point> points;
  for (int i = 0; i < contours_poly.size(); i++) {
    for (int j = 0; j < contours_poly[i].size(); j++) {
      points.push_back(Point(contours_poly[i][j].x, contours_poly[i][j].y));
    }
  }
  std::cout << points.size() << std::endl;
  cv::threshold(Image, Image, 10, 255, cv::THRESH_BINARY_INV);

  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                                boost::no_property,
                                boost::property<boost::edge_weight_t, double>>
      Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
  typedef std::pair<int, int> Edge;
  Graph m_graph;

  std::vector<cv::Point> GraphPoints;
  GraphPoints.push_back(cv::Point(-1, -1));

  cv::Point p0(-1, -1);
  cv::Point p1(-1, -1);
  cv::Mat Res = Image.clone();
  boost::polygon::voronoi_diagram<double> VD;
  construct_voronoi(points.begin(), points.end(), &VD);
  std::sort(contours_poly.begin(), contours_poly.end(), ContoursSort);
  for (boost::polygon::voronoi_diagram<double>::const_edge_iterator it =
           VD.edges().begin();
       it != VD.edges().end(); ++it) {
    if (it->is_primary()) {
      if (it->is_finite()) {

        double x0 = it->vertex0()->x();
        double y0 = it->vertex0()->y();

        double x1 = it->vertex1()->x();
        double y1 = it->vertex1()->y();

        cv::Point p_0(x0, y0);
        cv::Point p_1(x1, y1);

        int id0 = -1;
        int id1 = -1;
        if (p1 != p_0 && p0 != p_1) {
          p0 = p_0;
          p1 = p_1;
          if (x0 >= 0 && x1 >= 0 && y0 >= 0 && y1 >= 0 && x0 <= Image.cols &&
              x1 <= Image.cols && y0 <= Image.rows && y1 <= Image.rows) {
            if (Image.at<uchar>(y0, x0) == 0 && Image.at<uchar>(y1, x1) == 0) {

              for (int i = 0; i < GraphPoints.size(); i++) {
                if (p0 == GraphPoints[i]) {
                  id0 = i;
                  break;
                }
              }

              if (id0 == -1) {
                GraphPoints.push_back(p0);
                id0 = GraphPoints.size() - 1;
              }

              for (int i = 0; i < GraphPoints.size(); i++) {
                if (p1 == GraphPoints[i]) {
                  id1 = i;
                  break;
                }
              }

              if (id1 == -1) {
                GraphPoints.push_back(p1);
                id1 = GraphPoints.size() - 1;
              }
              double distance = powf((p0.x - p1.x), 2) + powf((p0.y - p1.y), 2);
              distance = sqrt(distance);
              boost::add_edge(id0, id1, distance, m_graph);

              cv::line(Mask, p0, p1, cv::Scalar(255), 1, 8);
              cv::line(Res, p0, p1, cv::Scalar(255), 1, 8);
            } else {
              int t = 0;
              /* cv::line(Mask, p0, p1, cv::Scalar(128), 1, cv::LINE_AA);
               cv::line(Res, p0, p1, cv::Scalar(128), 1, cv::LINE_AA);*/
            }
          } else {
            int t = 0;
          }
        }
      }
    }
  }

  double DisStart = 999999999;
  int IDStart = -1;
  double DisEnd = 999999999;
  int IDEnd = -1;
  for (int i = 0; i < GraphPoints.size(); i++) {

    double distanceStar =
        powf((GraphPoints[i].x - 750), 2) + powf((GraphPoints[i].y - 100), 2);
    double distanceEnd =
        powf((GraphPoints[i].x - 700), 2) + powf((GraphPoints[i].y - 3950), 2);
    distanceStar = sqrt(distanceStar);
    distanceEnd = sqrt(distanceEnd);
    if (distanceStar < DisStart) {
      DisStart = distanceStar;
      IDStart = i;
    }
    if (distanceEnd < DisEnd) {
      DisEnd = distanceEnd;
      IDEnd = i;
    }
  }

  int vStart = IDStart;
  int vEnd = IDEnd;
  std::vector<vertex_descriptor> p(boost::num_vertices(m_graph));
  std::vector<Graph::edge_descriptor> q(boost::num_vertices(m_graph));
  std::vector<double> d(boost::num_vertices(m_graph));
  vertex_descriptor s = boost::vertex(vStart, m_graph);
  boost::dijkstra_shortest_paths(
      m_graph, s,
      boost::predecessor_map(boost::make_iterator_property_map(
                                 p.begin(), get(boost::vertex_index, m_graph)))
          .distance_map(boost::make_iterator_property_map(
              d.begin(), get(boost::vertex_index, m_graph))));

  int t = vEnd;
  std::vector<int> path;
  for (; t != vStart; t = p[t])
    path.push_back(t);
  path.push_back(vEnd);
  reverse(path.begin(), path.end());

  cv::circle(Mask, GraphPoints[vStart], 3, cv::Scalar(255), 5, cv::LINE_8);
  cv::circle(Res, GraphPoints[vStart], 3, cv::Scalar(255), 5, cv::LINE_8);
  cv::circle(Mask, GraphPoints[vEnd], 3, cv::Scalar(255), 5, cv::LINE_8);
  cv::circle(Res, GraphPoints[vEnd], 3, cv::Scalar(255), 5, cv::LINE_8);

  for (int i = 1; i < path.size() - 2; i++) {
    cv::line(Mask, GraphPoints[path[i]], GraphPoints[path[i + 1]],
             cv::Scalar(255), 3, 8);
    cv::line(Res, GraphPoints[path[i]], GraphPoints[path[i + 1]],
             cv::Scalar(255), 3, 8);
  }

  long end_time = clock();
  std::cout << "Generate Product: " << end_time - begin_time << "ms\n";

  cv::imwrite("Mask.tif", Mask);
  cv::imwrite("Image.tif", Res);

  return 0;
}
