
#include <explore/costmap_tools.h>

namespace frontier_exploration
{

    std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D& costmap)
    {
        // get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out;

        unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) 
        {
            ROS_WARN("Evaluating nhood for offmap point");
            return out;
        }

        if (idx % size_x_ > 0) 
        {
            out.push_back(idx - 1);
        }
        if (idx % size_x_ < size_x_ - 1) 
        {
            out.push_back(idx + 1);
        }
        if (idx >= size_x_) 
        {
            out.push_back(idx - size_x_);
        }
        if (idx < size_x_ * (size_y_ - 1)) 
        {
            out.push_back(idx + size_x_);
        }
        return out;
    }


    std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D& costmap)
    {
        // get 8-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out = nhood4(idx, costmap);

        unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();

        if (idx > size_x_ * size_y_ - 1) 
        {
            return out;
        }

        if (idx % size_x_ > 0 && idx >= size_x_) 
        {
            out.push_back(idx - 1 - size_x_);
        }
        if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) 
        {
            out.push_back(idx - 1 + size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) 
        {
            out.push_back(idx + 1 - size_x_);
        }
        if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) 
        {
            out.push_back(idx + 1 + size_x_);
        }

        return out;
    }

    bool nearestCell(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap)
    {
        const unsigned char* map = costmap.getCharMap();
        const unsigned int size_x = costmap.getSizeInCellsX(), size_y = costmap.getSizeInCellsY();

        if (start >= size_x * size_y) {
            return false;
        }

        // initialize breadth first search
        std::queue<unsigned int> bfs;
        std::vector<bool> visited_flag(size_x * size_y, false);

        // push initial cell
        bfs.push(start);
        visited_flag[start] = true;

        // search for neighbouring cell matching value
        while (!bfs.empty()) 
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // return if cell of correct value is found
            if (map[idx] == val) 
            {
                result = idx;
                return true;
            }

            // iterate over all adjacent unvisited cells
            for (unsigned nbr : nhood8(idx, costmap)) 
            {
                if (!visited_flag[nbr]) 
                {
                    bfs.push(nbr);
                    visited_flag[nbr] = true;
                }
            }
        }
        return false;
    }

    bool nearestCellonArea(unsigned int& result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D& costmap, const std::vector<bool>& area_idx)
    {
        const unsigned char* map = costmap.getCharMap();
        const unsigned int size_x = costmap.getSizeInCellsX();
        const unsigned int size_y = costmap.getSizeInCellsY();

        if (start >= size_x * size_y) 
        {
            return false;
        }

        // initialize breadth first search
        std::queue<unsigned int> bfs;
        std::vector<bool> visited_flag(size_x * size_y, false);

        // push initial cell
        bfs.push(start);
        visited_flag[start] = true;

        // search for neighbouring cell matching value
        while (!bfs.empty()) 
        {
            unsigned int idx = bfs.front();
            bfs.pop();

            // return if cell of correct value is found
            if (map[idx] == val) 
            {
                result = idx;
                return true;
            }

            // iterate over all adjacent unvisited cells bellonging to chossen area
            for (unsigned nbr : nhood8(idx, costmap)) 
            {
                if (area_idx[nbr] && !visited_flag[nbr])
                {
                    bfs.push(nbr);
                    visited_flag[nbr] = true;
                }
            }
        }
        return false;
    }

    FrontierDivision divFrontier(geometry_msgs::Pose pose, const costmap_2d::Costmap2D& costmap) 
    {
        unsigned int size_x_ = costmap.getSizeInCellsX(), size_y_ = costmap.getSizeInCellsY();
        ROS_INFO("SIZE x: %u , y: %u",size_x_,size_y_);


        unsigned int pose_mx, pose_my;
        bool inBounds = costmap.worldToMap(pose.position.x,pose.position.y,pose_mx,pose_my);

        FrontierDivision frontierDiv;
        
        unsigned int idxPose;
        idxPose = costmap.getIndex(pose_mx,pose_my);

        if (idxPose > size_x_ * size_y_ - 1) {
            ROS_WARN("Evaluating divFrontier for offmap point");
            return frontierDiv;
        }
        
        ROS_INFO("ROBOT POSE INDEX -> %u", idxPose);
        
        unsigned int point_idx; 
        unsigned int mx, my;
        double wx, wy;
        std::vector<unsigned int> topRight;
        std::vector<unsigned int> topLeft;
        std::vector<unsigned int> bottomRight;
        std::vector<unsigned int> bottomLeft;
        geometry_msgs::Point point;
        point_idx = idxPose;
        // Bottom Right index points of the frontier division
        while (point_idx % size_x_ < size_x_ - 1 && point_idx >= size_x_)
        {
            point_idx = point_idx + 1 - size_x_;
            costmap.indexToCells(point_idx, mx, my);
            costmap.mapToWorld(mx, my, wx, wy);
            point.x = wx;
            point.y = wy;
            bottomRight.push_back(point_idx);
            frontierDiv.bottomRightPoints.push_back(point);
            
        }
        // Bottom Left index points of the frontier division
        point_idx = idxPose;
        while (point_idx % size_x_ > 0 && point_idx >= size_x_)
        {
            point_idx = point_idx - 1 - size_x_;
            costmap.indexToCells(point_idx, mx, my);
            costmap.mapToWorld(mx, my, wx, wy);
            point.x = wx;
            point.y = wy;
            bottomLeft.push_back(point_idx);
            frontierDiv.bottomLeftPoints.push_back(point);
        }
        //  Top Left index points of the frontier division
        point_idx = idxPose;
        while (point_idx % size_x_ > 0 && point_idx < size_x_ * (size_y_ - 1))
        {
            point_idx = point_idx - 1 + size_x_;
            costmap.indexToCells(point_idx, mx, my);
            costmap.mapToWorld(mx, my, wx, wy);
            point.x = wx;
            point.y = wy;
            topLeft.push_back(point_idx);
            frontierDiv.topLeftPoints.push_back(point);
        }
        // Top Right index points of the frontier division
        point_idx = idxPose;
        while (point_idx % size_x_ < size_x_ - 1 && point_idx < size_x_ * (size_y_ - 1))
        {
            point_idx = point_idx + 1 + size_x_;
            costmap.indexToCells(point_idx, mx, my);
            costmap.mapToWorld(mx, my, wx, wy);
            point.x = wx;
            point.y = wy;
            topRight.push_back(point_idx);
            frontierDiv.topRightPoints.push_back(point);
        }
        

        std::vector<bool> topArea_flag(size_x_ * size_y_, false);
        std::vector<bool> leftArea_flag(size_x_ * size_y_, false);
        std::vector<bool> rightArea_flag(size_x_ * size_y_, false);
        std::vector<bool> bottomArea_flag(size_x_ * size_y_, false);

        // Right Area Frontier 
        unsigned int right_idx; 
        for(unsigned int topRightidx : topRight) 
        {
            // Top Right Diagonal will be added to Right Area
            rightArea_flag[topRightidx] = true;
            
            right_idx = topRightidx;
            while (right_idx % size_x_ < size_x_ - 1) 
            {
                right_idx++;
                rightArea_flag[right_idx]=true;
            }
        }
        for(unsigned int bottomRightidx : bottomRight) 
        {
            right_idx = bottomRightidx;
            while (right_idx % size_x_ < size_x_ - 1) 
            {
                right_idx++;
                rightArea_flag[right_idx]=true;
            }
        }
        right_idx = idxPose;
        while ( right_idx % size_x_ < size_x_ - 1) 
        {
            right_idx++;
            rightArea_flag[right_idx]=true;
        }

        // Top Area Frontier
        unsigned int top_idx;
        for(unsigned int topRightidx : topRight) 
        {
            top_idx = topRightidx;
            while (top_idx < size_x_ * (size_y_ - 1)) 
            {
                top_idx = top_idx + size_x_;
                topArea_flag[top_idx]=true;
            }
        }
        for(unsigned int topLeftidx : topLeft) 
        {
            // Top Left Diagonal will be added to top Area
            topArea_flag[topLeftidx] = true;

            top_idx = topLeftidx;
            while (top_idx < size_x_ * (size_y_ - 1)) 
            {
                top_idx = top_idx + size_x_;
                topArea_flag[top_idx]=true;
            }
        }
        top_idx = idxPose;
        while (top_idx < size_x_ * (size_y_ - 1)) 
        {
            top_idx = top_idx + size_x_;
            topArea_flag[top_idx]=true;
        }

        // Left Area Frontier
        unsigned int left_idx;
        for(unsigned int topLeftidx : topLeft) 
        {
            left_idx = topLeftidx;
            while (left_idx % size_x_ > 0) 
            {
                left_idx--;
                leftArea_flag[left_idx]=true;
            }
        }
        for(unsigned int bottomLeftidx : bottomLeft)
        {   
            // Bottom Left Diagonal will be added to Left Area
            leftArea_flag[bottomLeftidx] = true;

            left_idx = bottomLeftidx;
            while (left_idx % size_x_ > 0)
            {
                left_idx--;
                leftArea_flag[left_idx]=true;
            }
        }
        left_idx = idxPose;
        while (left_idx % size_x_ > 0)
        {
            left_idx--;
            leftArea_flag[left_idx]=true;
        }

        // Bottom Area Frontier
        unsigned int bottom_idx;
        for(unsigned int bottomLeftidx : bottomLeft)
        {
            bottom_idx = bottomLeftidx;
            while (bottom_idx >= size_x_)
            {
                bottom_idx = bottom_idx - size_x_;
                bottomArea_flag[bottom_idx]=true;
            }
        } 
        for(unsigned int bottomRightidx : bottomRight)
        {
            // Bottom right Diagonal added to Bottom Area
            bottomArea_flag[bottomRightidx] = true;

            bottom_idx = bottomRightidx;
            while (bottom_idx >= size_x_)
            {
                bottom_idx = bottom_idx - size_x_;
                bottomArea_flag[bottom_idx]=true;
            }
        }
        bottom_idx = idxPose;
        while (bottom_idx >= size_x_)
        {
            bottom_idx = bottom_idx - size_x_;
            bottomArea_flag[bottom_idx]=true;
        }
        
        frontierDiv.rightArea = rightArea_flag;
        frontierDiv.topArea = topArea_flag;
        frontierDiv.leftArea = leftArea_flag;
        frontierDiv.bottomArea = bottomArea_flag;

        frontierDiv.rightAreaPoints = getAreaPoints(rightArea_flag, costmap);
        frontierDiv.topAreaPoints = getAreaPoints(topArea_flag, costmap);
        frontierDiv.leftAreaPoints = getAreaPoints(leftArea_flag, costmap);
        frontierDiv.bottomAreaPoints = getAreaPoints(bottomArea_flag, costmap);

        return frontierDiv;
    }

    std::vector<geometry_msgs::Point> getAreaPoints(std::vector<bool> area_idx, const costmap_2d::Costmap2D& costmap)
    {
        std::vector<geometry_msgs::Point> out;
        size_t num_area_idx = area_idx.size();
        unsigned int idx;
        unsigned int mx, my;
        double wx, wy;
        geometry_msgs::Point point;
        for (idx = 0; idx < num_area_idx ; ++idx)
        {
            if (area_idx[idx])
            {
                costmap.indexToCells(idx, mx, my);
                costmap.mapToWorld(mx, my, wx, wy);
                point.x = wx;
                point.y = wy;
                out.push_back(point);
            }
        }
        return out;
    }

}