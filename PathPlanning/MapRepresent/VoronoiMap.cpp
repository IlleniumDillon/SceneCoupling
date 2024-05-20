#include "VoronoiMap.hpp"

VoronoiMap::VoronoiMap()
{
    this->vertices.clear();
}

VoronoiMap::VoronoiMap(const VoronoiMap &other)
{
    this->vertices = other.vertices;
}

// VoronoiMap::VoronoiMap(const Scene &scene)
// {
// }

VoronoiMap::VoronoiMap(OccupancyGridMap &map)
{
    int **delicateMap = new int *[map.indexHeight];
    for (int i = 0; i < map.indexHeight; i++)
    {
        delicateMap[i] = new int[map.indexWidth];
    }
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            delicateMap[i][j] = 0;
        }
    }
    /// step 1: delicat the map till no more dilation is needed
    int count = delicateAndCount(delicateMap, map);

#if PREVIEW_FLAG
    cv::Mat img_step1 = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC1);
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            img_step1.at<uchar>(i, j) = delicateMap[i][j];
        }
    }
    cv::imshow("delicate", img_step1);
    cv::waitKey(0);
#endif

    /// step 2: mark all local maximums as vertices
    std::vector<Eigen::Vector2i> backBone;
    getBackBone(delicateMap, map, backBone);

#if PREVIEW_FLAG
    cv::Mat img_step2 = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (map(j, i) == 0)
            {
                img_step2.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            }
            else
            {
                img_step2.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    for (int i = 0; i < backBone.size(); i++)
    {
        img_step2.at<cv::Vec3b>(backBone[i](1), backBone[i](0)) = cv::Vec3b(0, 0, 255);
    }
    cv::imshow("backBone", img_step2);
    cv::waitKey(0);
#endif

    /// step 3: simplify the vertices and link them with edges

    /// step 4: generate the Voronoi diagram

    /// step 5: delete the delicate map
    for (int i = 0; i < map.indexHeight; i++)
    {
        delete[] delicateMap[i];
    }
    delete[] delicateMap;
}

VoronoiMap &VoronoiMap::operator=(const VoronoiMap &other)
{
    if (this == &other)
    {
        return *this;
    }
    this->vertices = other.vertices;
    return *this;
}

VoronoiMap::~VoronoiMap()
{
    this->vertices.clear();
}

int VoronoiMap::delicateAndCount(int **delicateMap, OccupancyGridMap &map)
{
    auto clearMap = [](uint8_t** tempMap, OccupancyGridMap &map) {
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                tempMap[i][j] = 0;
            }
        }
    };
    auto copyMap = [](uint8_t** tempMap, OccupancyGridMap &map) {
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                tempMap[i][j] = map(j,i);
            }
        }
    };
    auto checkZero = [](uint8_t** tempMap, OccupancyGridMap &map) {
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                if (tempMap[i][j] != 0)
                {
                    return false;
                }
            }
        }
        return true;
    };
    int count = 0;
    uint8_t** tempMap = new uint8_t*[map.indexHeight];
    for (int i = 0; i < map.indexHeight; i++)
    {
        tempMap[i] = new uint8_t[map.indexWidth];
    }
    uint8_t** diff = new uint8_t*[map.indexHeight];
    for (int i = 0; i < map.indexHeight; i++)
    {
        diff[i] = new uint8_t[map.indexWidth];
    }

    copyMap(tempMap, map);
    // clearMap(diff, map);

    do
    {
        clearMap(diff, map);
        count++;
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                if (tempMap[i][j] == 1)
                {
                    continue;
                }
                /// if threre is more than one 1 in the 3x3 window, then set the center to 1
                int occ = 0;
                for (int m = i - 1; m <= i + 1; m++)
                {
                    for (int n = j - 1; n <= j + 1; n++)
                    {
                        if (m < 0 || m >= map.indexHeight || n < 0 || n >= map.indexWidth)
                        {
                            continue;
                        }
                        if (tempMap[m][n] == 1)
                        {
                            occ++;
                            break;
                        }
                    }
                }
                if (occ != 0)
                {
                    diff[i][j] = 1;
                }
            }
        }
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                if (diff[i][j] != 0)
                {
                    tempMap[i][j] = 1;
                    delicateMap[i][j] = diff[i][j] * count;
                }
            }
        }
    }while(!checkZero(diff, map));
    
    // delete the map
    for (int i = 0; i < map.indexHeight; i++)
    {
        delete[] tempMap[i];
    }
    delete[] tempMap;
    for (int i = 0; i < map.indexHeight; i++)
    {
        delete[] diff[i];
    }
    delete[] diff;
    return count;
}

int VoronoiMap::getBackBone(int **delicateMap, OccupancyGridMap &map, std::vector<Eigen::Vector2i> &backBone)
{
    auto checkIsBackBone = [](int p0, int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8) -> bool {
        /// test 1
        // if (
        //     ((p4 > p0 && p4 > p1 && p4 > p3) && (p4 <= p5 && p4 <= p7 && p4 <= p8)) ||
        //     ((p4 <= p0 && p4 <= p1 && p4 <= p3) && (p4 > p5 && p4 > p7 && p4 > p8)) ||
        //     ((p4 > p1 && p4 > p2 && p4 > p5) && (p4 <= p3 && p4 <= p6 && p4 <= p7)) ||
        //     ((p4 <= p1 && p4 <= p2 && p4 <= p5) && (p4 > p3 && p4 > p6 && p4 > p7)) ||

        //     (p4 > p3 && p4 >= p5 && p1 > p0 && p1 >= p2 && p7 > p6 && p7 >= p8) ||
        //     (p4 > p1 && p4 >= p7 && p3 > p0 && p3 >= p6 && p5 > p2 && p5 >= p8) 
        // )
        // {
        //     return true;
        // }

        uint8_t condition_1 = (p4 >= p0 && p4 >= p8) ? 1 : 0;
        uint8_t condition_2 = (p4 >= p2 && p4 >= p6) ? 1 : 0;
        uint8_t condition_3 = (p4 >= p1 && p4 >= p7) ? 1 : 0;
        uint8_t condition_4 = (p4 >= p3 && p4 >= p5) ? 1 : 0;

        uint8_t condition_5 = (p4 > p0 && p4 > p8) ? 2 : 0;
        uint8_t condition_6 = (p4 > p2 && p4 > p6) ? 2 : 0;
        uint8_t condition_7 = (p4 > p1 && p4 > p7) ? 2 : 0;
        uint8_t condition_8 = (p4 > p3 && p4 > p5) ? 2 : 0;

        uint8_t condition = condition_1 + condition_2 + condition_3 + condition_4 + condition_5 + condition_6 + condition_7 + condition_8;

        if (condition >= 4)
        {
            return true;
        }

        return false;
    };

    cv::Mat img = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC1);

    for (int i = 1; i < map.indexHeight - 1; i++)
    {
        for (int j = 1; j < map.indexWidth - 1; j++)
        {
            int p0 = delicateMap[i - 1][j - 1];
            int p1 = delicateMap[i - 1][j];
            int p2 = delicateMap[i - 1][j + 1];
            int p3 = delicateMap[i][j - 1];
            int p4 = delicateMap[i][j];
            int p5 = delicateMap[i][j + 1];
            int p6 = delicateMap[i + 1][j - 1];
            int p7 = delicateMap[i + 1][j];
            int p8 = delicateMap[i + 1][j + 1];
            if (checkIsBackBone(p0, p1, p2, p3, p4, p5, p6, p7, p8) && p4 > 10)
            {
                backBone.push_back(Eigen::Vector2i(j, i));
                img.at<uchar>(i, j) = 255;
            }
        }
    }

    /// post processing
    /// dilate
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat temp1, temp2;
    cv::morphologyEx(img, img, cv::MORPH_CLOSE, element);
    //cv::morphologyEx(img, temp2, cv::MORPH_BLACKHAT, element);

    /// copy the result to the backBone
    backBone.clear();
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (img.at<uchar>(i, j) != 0)
            {
                backBone.push_back(Eigen::Vector2i(j, i));
            }
        }
    }

    // cv::imshow("second derivative", img);
    // cv::imwrite("second_derivative.png", img);
    // cv::waitKey(0);
    return 0;
}

int VoronoiMap::generateVertices(int **delicateMap, OccupancyGridMap &map, std::vector<Eigen::Vector2i> &backBone)
{
    /// step 1: generate vertices list from backBone
    std::vector<vertex> vertices;

    /// step 2: do the following when verices is not empty
    while (!vertices.empty())
    {
        /// step 2-1: find a vertex with only one neighbor and put it into the open list
       
        
    }
    return 0;
}
