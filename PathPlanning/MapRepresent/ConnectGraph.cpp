#include "ConnectGraph.hpp"

ConnectGraph::ConnectGraph()
{
    connectGridMap = nullptr;
    indexWidth = 0;
    indexHeight = 0;
    connectAreaNum = 0;
}

ConnectGraph::ConnectGraph(const ConnectGraph &other)
{
    connectGridMap = new int8_t *[other.indexHeight];
    for (int i = 0; i < other.indexHeight; i++)
    {
        connectGridMap[i] = new int8_t[other.indexWidth];
        for (int j = 0; j < other.indexWidth; j++)
        {
            connectGridMap[i][j] = other.connectGridMap[i][j];
        }
    }
    indexWidth = other.indexWidth;
    indexHeight = other.indexHeight;
    connectAreaNum = other.connectAreaNum;
}

ConnectGraph::ConnectGraph(OccupancyGridMap &map)
{
    connectGridMap = new int8_t *[map.indexHeight];
    for (int i = 0; i < map.indexHeight; i++)
    {
        connectGridMap[i] = new int8_t[map.indexWidth];
        for (int j = 0; j < map.indexWidth; j++)
        {
            connectGridMap[i][j] = -1;
        }
    }
    indexWidth = map.indexWidth;
    indexHeight = map.indexHeight;
    connectAreaNum = 0;

    generateConnectGridMap(map);

#if PREVIEW_FLAG
    cv::Mat show = cv::Mat::zeros(map.indexHeight, map.indexWidth, CV_8UC3);

    std::vector<cv::Vec3b> colors;
    for (int i = 0; i < connectAreaNum; i++)
    {
        cv::RNG rng(cv::getTickCount());
        colors.push_back(cv::Vec3b(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)));
    }

    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (connectGridMap[i][j] == 0)
            {
                show.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
            else if (connectGridMap[i][j] == -1)
            {
                show.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128);
            }
            else
            {
                show.at<cv::Vec3b>(i, j) = colors[connectGridMap[i][j] - 1];
            }
        }
    }
    cv::imshow("connectGridMap", show);
    cv::waitKey(0);
#endif
}

ConnectGraph &ConnectGraph::operator=(const ConnectGraph &other)
{
    if (this == &other)
    {
        return *this;
    }
    if (connectGridMap != nullptr)
    {
        for (int i = 0; i < indexHeight; i++)
        {
            delete[] connectGridMap[i];
        }
        delete[] connectGridMap;
    }
    connectGridMap = new int8_t *[other.indexHeight];
    for (int i = 0; i < other.indexHeight; i++)
    {
        connectGridMap[i] = new int8_t[other.indexWidth];
        for (int j = 0; j < other.indexWidth; j++)
        {
            connectGridMap[i][j] = other.connectGridMap[i][j];
        }
    }
    indexWidth = other.indexWidth;
    indexHeight = other.indexHeight;
    connectAreaNum = other.connectAreaNum;
    return *this;
}

ConnectGraph::~ConnectGraph()
{
    if (connectGridMap != nullptr)
    {
        for (int i = 0; i < indexHeight; i++)
        {
            delete[] connectGridMap[i];
        }
        delete[] connectGridMap;
    }
}

int ConnectGraph::generateConnectGridMap(OccupancyGridMap &map)
{
    //cv::Mat show = cv::Mat(map.indexHeight, map.indexWidth, CV_8UC3, cv::Scalar(128, 128, 128));
    int totalGrid = map.indexHeight * map.indexWidth;
    // set obstacle
    for (int i = 0; i < map.indexHeight; i++)
    {
        for (int j = 0; j < map.indexWidth; j++)
        {
            if (map.map[i][j] == 1)
            {
                connectGridMap[i][j] = 0;
                totalGrid--;
                //show.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    while (totalGrid > 0)
    {
        std::vector<Eigen::Vector2i> openlist;
        /// find a unknown grid
        Eigen::Vector2i seed = Eigen::Vector2i(-1, -1);
        for (int i = 0; i < indexHeight; i++)
        {
            for (int j = 0; j < indexWidth; j++)
            {
                if (connectGridMap[i][j] == -1)
                {
                    seed = Eigen::Vector2i(i, j);
                    break;
                }
            }
            if (seed != Eigen::Vector2i(-1, -1))
            {
                break;
            }
        }
        if (seed == Eigen::Vector2i(-1, -1))
        {
            break;
        }

        /// random color
        //cv::RNG rng(cv::getTickCount());
        //cv::Vec3b color = cv::Vec3b(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

        connectAreaNum++;
        openlist.push_back(seed);

        while (!openlist.empty())
        {
            Eigen::Vector2i current = openlist.back();
            openlist.pop_back();
            connectGridMap[current.x()][current.y()] = connectAreaNum;
            totalGrid--;
            //show.at<cv::Vec3b>(current.x(), current.y()) = color;

            for (int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j++)
                {
                    if (i == 0 && j == 0)
                    {
                        continue;
                    }
                    Eigen::Vector2i neighbor = current + Eigen::Vector2i(i, j);
                    if (neighbor.x() >= 0 && neighbor.x() < indexHeight && neighbor.y() >= 0 && neighbor.y() < indexWidth)
                    {
                        if (connectGridMap[neighbor.x()][neighbor.y()] == -1)
                        {
                            openlist.push_back(neighbor);
                            connectGridMap[neighbor.x()][neighbor.y()] = -2;
                        }
                    }
                }
            }

            //cv::imshow("connectGridMap", show);
            //cv::waitKey(1);
        }
        std::cout << "totalGrid: " << totalGrid << std::endl;
    }
    
    std::cout << "connectAreaNum: " << connectAreaNum << std::endl;

    return 0;
}
