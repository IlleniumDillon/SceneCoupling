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
    cv::imwrite("delicate.png", img_step1);
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
    cv::imwrite("backBone.png", img_step2);
    cv::waitKey(0);
#endif

    /// step 3: simplify the vertices and link them with edges
    std::vector<Eigen::Vector2i> verticesList;
    generateVertices(verticesList, map, backBone);
    /// step 4: generate the Voronoi diagram
    generateGraph(verticesList, map);
#if PREVIEW_FLAG
    cv::Mat img_step4 = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    for (int i = 0; i < vertices.size(); i++)
    {
        for (int j = 0; j < vertices[i]->neighbors.size(); j++)
        {
            Eigen::Vector2i start = vertices[i]->position.cast<int>();
            Eigen::Vector2i end = vertices[i]->neighbors[j]->position.cast<int>();
            std::vector<Eigen::Vector2i> line = drawLine(start, end);
            for (int k = 0; k < line.size(); k++)
            {
                img_step4.at<cv::Vec3b>(line[k](1), line[k](0)) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    cv::imshow("graph", img_step4);
    cv::waitKey(0);
#endif
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
                    if (occ != 0)
                    {
                        break;
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
            if (checkIsBackBone(p0, p1, p2, p3, p4, p5, p6, p7, p8) && p4 > CENTER_DISTANCE)
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
#if PREVIEW_FLAG
    cv::imshow("second derivative", img);
    cv::imwrite("second_derivative.png", img);
    cv::waitKey(0);
#endif
    return 0;
}

int VoronoiMap::generateVertices(std::vector<Eigen::Vector2i>& verticesList, OccupancyGridMap &map, std::vector<Eigen::Vector2i> &backBone)
{
    /// step 1: generate node map from backBone
    VoronoiGridNode **nodeMap = new VoronoiGridNode *[map.indexHeight];
    for (int i = 0; i < map.indexHeight; i++)
    {
        nodeMap[i] = new VoronoiGridNode[map.indexWidth];
        for (int j = 0; j < map.indexWidth; j++)
        {
            nodeMap[i][j].position = Eigen::Vector2i(j, i);
            nodeMap[i][j].flag = 0;
            nodeMap[i][j].numOfNeighbors = 0;
            nodeMap[i][j].comeForm = nullptr;
            nodeMap[i][j].isBackBone = false;
        }
    }
    for (int i = 0; i < backBone.size(); i++)
    {
        nodeMap[backBone[i](1)][backBone[i](0)].isBackBone = true;
    }
    for (int i = 0; i < backBone.size(); i++)
    {
        int x = backBone[i](0);
        int y = backBone[i](1);
        for (int m = y - 1; m <= y + 1; m++)
        {
            for (int n = x - 1; n <= x + 1; n++)
            {
                if (m < 0 || m >= map.indexHeight || n < 0 || n >= map.indexWidth)
                {
                    continue;
                }
                if (m == y && n == x)
                {
                    continue;
                }
                if (!nodeMap[m][n].isBackBone)
                {
                    continue;
                }
                nodeMap[m][n].numOfNeighbors++;
            }
        }
    }

    /// step 2: do the following till no more vertices can be generated
    std::vector<Eigen::Vector2i> verticesList_temp;
    int backBoneSize = backBone.size();
    std::vector<VoronoiGridNode *> openList;
    cv::Mat show = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    while (backBoneSize > 0)
    {
        openList.clear();
        /// step 2-1: find a vertex with only one neighbor and put it into the open list
        for (int i = 0; i < map.indexHeight; i++)
        {
            for (int j = 0; j < map.indexWidth; j++)
            {
                if (nodeMap[i][j].isBackBone && nodeMap[i][j].numOfNeighbors == 1 && nodeMap[i][j].flag == 0)
                {
                    nodeMap[i][j].flag = 1;
                    nodeMap[i][j].comeForm = nullptr;
                    openList.push_back(&nodeMap[i][j]);
                    break;
                }
            }
            if (!openList.empty())
            {
                break;
            }
        }
        /// step 2-2: BFS search
        // std::vector<Eigen::Vector2i> vertices_temp;
        
        while (!openList.empty())
        {
            // VoronoiGridNode *node = openList.back();
            VoronoiGridNode *node = openList.front();
            node->flag = -1;
            backBoneSize--;
            //  openList.pop_back();
            openList.erase(openList.begin());
            show.at<cv::Vec3b>(node->position(1), node->position(0)) = cv::Vec3b(255, 255, 255);
            int x = node->position(0);
            int y = node->position(1);
            bool critical = false;
            for (int m = y - 1; m <= y + 1; m++)
            {
                for (int n = x - 1; n <= x + 1; n++)
                {
                    if (m < 0 || m >= map.indexHeight || n < 0 || n >= map.indexWidth)
                    {
                        continue;
                    }
                    if (!nodeMap[m][n].isBackBone)
                    {
                        continue;
                    }
                    if (nodeMap[m][n].flag != 0)
                    {
                        continue;
                    }
                    nodeMap[m][n].flag = 1;
                    nodeMap[m][n].comeForm = node;
                    openList.push_back(&nodeMap[m][n]);
                    if (node->numOfNeighbors == 1 || node->numOfNeighbors != nodeMap[m][n].numOfNeighbors)
                    {
                        int minDistance = std::numeric_limits<int>::max();
                        int index = -1;
                        for (int i = 0; i < verticesList_temp.size(); i++)
                        {
                            int distance = (verticesList_temp[i] - node->position).norm();
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                index = i;
                            }
                        }
                        if (minDistance > CENTER_DISTANCE)
                        {
                            critical = true;
                        }
                        else
                        {
                            node->comeForm = &nodeMap[verticesList_temp[index](1)][verticesList_temp[index](0)];
                            nodeMap[m][n].comeForm = &nodeMap[verticesList_temp[index](1)][verticesList_temp[index](0)];
                        }
                    }
                    //break;
                }
            }
            if (critical)
            {
                verticesList_temp.push_back(node->position);
                // vertexPtr v = std::make_shared<vertex>();
                // v->position = node->position.cast<double>();
                // VoronoiGridNode *temp = node;
                // vertexPtr comeForm = nullptr;
                // while (temp->comeForm != nullptr)
                // {
                //     temp = temp->comeForm;
                //     for (int i = 0; i < vertices.size(); i++)
                //     {
                //         if (vertices[i]->position == temp->position.cast<double>())
                //         {
                //             comeForm = vertices[i];
                //             break; 
                //         }
                //     }
                //     if (comeForm != nullptr)
                //     {
                //         break;
                //     }
                // }
                // if (comeForm != nullptr)
                // {
                //     v->neighbors.push_back(comeForm);
                //     comeForm->neighbors.push_back(v);
                //     vertices.push_back(v);
                // }
                // else
                // {
                //     vertices.push_back(v);
                // }
            }

            cv::imshow("show", show);
            cv::waitKey(1);
        }
        // std::cout << "backBoneSize: " << backBoneSize << std::endl;
    }

#if PREVIEW_FLAG
    cv::Mat img = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);
    for (int i = 0; i < verticesList_temp.size(); i++)
    {
        img.at<cv::Vec3b>(verticesList_temp[i](1), verticesList_temp[i](0)) = cv::Vec3b(255, 255, 255);
    }
    for (int i = 0; i < vertices.size(); i++)
    {
        for (int j = 0; j < vertices[i]->neighbors.size(); j++)
        {
            Eigen::Vector2i start = vertices[i]->position.cast<int>();
            Eigen::Vector2i end = vertices[i]->neighbors[j]->position.cast<int>();
            std::vector<Eigen::Vector2i> line = drawLine(start, end);
            for (int k = 0; k < line.size(); k++)
            {
                img.at<cv::Vec3b>(line[k](1), line[k](0)) = cv::Vec3b(255, 255, 255);
            }
        }
    }
    cv::imshow("vertices", img);
    cv::imwrite("vertices.png", img);
    cv::waitKey(0);
#endif
    verticesList = verticesList_temp;
    /// step 3: delete the node map
    for (int i = 0; i < map.indexHeight; i++)
    {
        delete[] nodeMap[i];
    }
    delete[] nodeMap;
    return 0;
}

int VoronoiMap::generateGraph(std::vector<Eigen::Vector2i> &verticesList, OccupancyGridMap &map)
{
    /// check any two vertices
    for (int i = 0; i < verticesList.size(); i++)
    {
        for (int j = i + 1; j < verticesList.size(); j++)
        {
            Eigen::Vector2i start = verticesList[i];
            Eigen::Vector2i end = verticesList[j];
            std::vector<Eigen::Vector2i> line = drawLine(start, end);
            if (checkLineOccupancy(line, map))
            {
                continue;
            }
            vertexPtr v1 = nullptr;
            vertexPtr v2 = nullptr;
            for (int k = 0; k < vertices.size(); k++)
            {
                if (vertices[k]->position == start.cast<double>())
                {
                    v1 = vertices[k];
                }
                if (vertices[k]->position == end.cast<double>())
                {
                    v2 = vertices[k];
                }
            }
            if (v1 == nullptr)
            {
                v1 = std::make_shared<vertex>();
                v1->position = start.cast<double>();// * map.resolution + Eigen::Vector2d(map.origin(0), map.origin(1)); 
                vertices.push_back(v1);
            }
            if (v2 == nullptr)
            {
                v2 = std::make_shared<vertex>();
                v2->position = end.cast<double>();// * map.resolution + Eigen::Vector2d(map.origin(0), map.origin(1));
                vertices.push_back(v2);
            }
            v1->neighbors.push_back(v2);
            v2->neighbors.push_back(v1);
        }
    }
    return 0;
}

std::vector<Eigen::Vector2i> VoronoiMap::drawLine(Eigen::Vector2i start, Eigen::Vector2i end)
{
    std::vector<Eigen::Vector2i> line;
    int x0 = start(0);
    int y0 = start(1);
    int x1 = end(0);
    int y1 = end(1);
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true)
    {
        line.push_back(Eigen::Vector2i(x0, y0));
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    return line;
}

bool VoronoiMap::checkLineOccupancy(std::vector<Eigen::Vector2i> points, OccupancyGridMap &map)
{
    for (int i = 0; i < points.size(); i++)
    {
        if (map(points[i](0), points[i](1)) == 1)
        {
            return true;
        }
    }
    return false;
}
