#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>

class my_local_planner{

    private:
        ros::NodeHandle nh;
        ros::Subscriber map_sub,pos_sub,path_sub;
        ros::Publisher my_local_map_pub,vis_pub,Square_Array_pub,ESDF_pub;
        ros::Timer gen_local_map,find_obs;    
        std::vector<double> tmp_buffer1;
        std::vector<double> my_local_map_neg;
        std::vector<double> esdf_dis;
        std::vector<double> esdf_dis_neg;
        // std::vector<double> esdf_dis_;   
        visualization_msgs::MarkerArray ESDF_point_Array;
        bool map_receive_flag = false;
        bool pos_receive_flag = false;
        bool my_local_map_flag = false;
        bool re_path_flag = false;
        // 连通域结构体，用于存储连通域的信息
        struct ConnectedComponent {
        int id;               // 连通域的ID
        std::vector<int> cells; // 连通域中包含的栅格单元
        };

    public:
        Eigen::Vector3d pos;
        nav_msgs::OccupancyGrid map,my_local_map;
        nav_msgs::Path re_path;
        std::vector<double> my_local_esdf_map;
        std::vector<double> global_esdf_map;
        double local_map_width = 6;
        double local_map_length = 6;
        double Square_width = 1;
        double dt = 0.5;
        double resolution = 0.05;

        my_local_planner(){ 
            esdf_dis.reserve(40000000);
            tmp_buffer1.reserve(40000000);
            my_local_esdf_map.resize(local_map_width*local_map_length/(resolution*resolution));
            // esdf_dis_.reserve(40000000);
            map_sub = nh.subscribe("/map",1,&my_local_planner::map_callback,this);
            pos_sub = nh.subscribe<geometry_msgs::Point>("/simu_pos",1,&my_local_planner::pos_callback,this);
            path_sub = nh.subscribe<nav_msgs::Path>("/AStar_path",1,&my_local_planner::path_callback,this);
            my_local_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/my_local_map",10);
            Square_Array_pub = nh.advertise<visualization_msgs::MarkerArray>("/Square_Array",10);
            ESDF_pub = nh.advertise<visualization_msgs::MarkerArray>("/ESDF_map",10);
            gen_local_map = nh.createTimer(ros::Duration(dt),boost::bind(&my_local_planner::simu_timer_callback,this,_1));
            find_obs = nh.createTimer(ros::Duration(dt),boost::bind(&my_local_planner::find_obs_callback,this,_1));
            vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/vis_obs", 0);            
        }

        void path_callback(const nav_msgs::Path::ConstPtr& msg){
            if(my_local_map_flag){
                re_path = *msg;
                re_path_flag = true;

                std::vector<std::vector<double>> path_xy;
                std::vector<double> last_xy;
                for(int k = 0;k < re_path.poses.size();++k){
                    if(k==0){
                        std::vector<double> xy;
                        xy.push_back(re_path.poses[k].pose.position.x);
                        xy.push_back(re_path.poses[k].pose.position.y);
                        path_xy.push_back(xy);
                        last_xy = xy;
                    }
                    else{
                        std::vector<double> xy;
                        if(std::abs(re_path.poses[k].pose.position.x - pos(0)) >= local_map_width/2 || std::abs(re_path.poses[k].pose.position.y - pos(1)) >= local_map_length/2)
                            break;
                        xy.push_back(re_path.poses[k].pose.position.x);
                        xy.push_back(re_path.poses[k].pose.position.y);
                        double dis = std::sqrt((xy[0]-last_xy[0])*(xy[0]-last_xy[0]) + (xy[1]-last_xy[1])*(xy[1]-last_xy[1]));
                        if(dis > Square_width)
                        {
                            path_xy.push_back(xy);
                            last_xy = xy;
                        }                          
                    }
                } 
                // int Square_step_half = std::floor(Square_width/2/my_local_map.info.resolution);
                visualization_msgs::MarkerArray MarkerArray;
                for(int k = 0;k < path_xy.size();++k){
                    visualization_msgs::Marker mark;
                    for(double i = Square_width/2;i >= my_local_map.info.resolution;i -= my_local_map.info.resolution){
                        double start_x = path_xy[k][0] - i;
                        double start_y = path_xy[k][1] - i;
                        for(;start_y <= path_xy[k][1] + i; start_y += my_local_map.info.resolution){
                            std::vector<double> xy;
                            xy.push_back(start_x);
                            xy.push_back(start_y);
                            int idx = find_my_local_map_idx(xy);
                            int val = my_local_map.data[idx];
                            if(val == 1)
                                break;
                        }
                        if(start_y < path_xy[k][1] + i)
                            continue;
                        start_x = path_xy[k][0] + i;
                        start_y = path_xy[k][1] - i;
                        for(;start_y <= path_xy[k][1] + i; start_y += my_local_map.info.resolution){
                            std::vector<double> xy;
                            xy.push_back(start_x);
                            xy.push_back(start_y);
                            if(my_local_map.data[find_my_local_map_idx(xy)] == 1)
                                break;
                        }
                        if(start_y < path_xy[k][1] + i)
                            continue;
                        //
                        //
                        //
                        start_x = path_xy[k][0] - i;
                        start_y = path_xy[k][1] - i;
                        for(;start_x <= path_xy[k][0] + i; start_x += my_local_map.info.resolution){
                            std::vector<double> xy;
                            xy.push_back(start_x);
                            xy.push_back(start_y);
                            if(my_local_map.data[find_my_local_map_idx(xy)] == 1)
                                break;
                        }
                        if(start_x < path_xy[k][0] + i)
                            continue;
                        start_x = path_xy[k][0] - i;
                        start_y = path_xy[k][1] + i;
                        for(;start_x <= path_xy[k][0] + i; start_x += my_local_map.info.resolution){
                            std::vector<double> xy;
                            xy.push_back(start_x);
                            xy.push_back(start_y);
                            if(my_local_map.data[find_my_local_map_idx(xy)] == 1)
                                break;
                        }
                        if(start_x < path_xy[k][0] + i)
                            continue;
                        geometry_msgs::Point point[4];
                        point[0].x = path_xy[k][0] - i;point[0].y = path_xy[k][1] - i;
                        point[1].x = path_xy[k][0] - i;point[1].y = path_xy[k][1] + i;
                        point[2].x = path_xy[k][0] + i;point[2].y = path_xy[k][1] + i;
                        point[3].x = path_xy[k][0] + i;point[3].y = path_xy[k][1] - i;
                        mark.header.frame_id = "map";
                        mark.header.stamp = ros::Time::now();
                        mark.ns = "Square";
                        mark.action = visualization_msgs::Marker::ADD;
                        mark.pose.orientation.w = 1.0;
                        mark.id = k; //unique id, useful when multiple markers exist.
                        mark.type = visualization_msgs::Marker::LINE_STRIP; //marker type
                        mark.scale.x = 0.05; //width of the line
                        mark.color.r = 1.0; mark.color.g = 0; mark.color.b = 0;//color of the line: white.
                        mark.color.a = 1.0; //不透明度，设0则全透明
                        mark.points.push_back(point[0]);
                        mark.points.push_back(point[1]);
                        mark.points.push_back(point[2]);
                        mark.points.push_back(point[3]);
                        mark.points.push_back(point[0]);
                        MarkerArray.markers.push_back(mark);
                        break;
                        // ROS_INFO("The square has been found!");
                    }                   
                } 
                Square_Array_pub.publish(MarkerArray);          
            }
            else{
                ROS_WARN("my_local_planner: The path has not received!");
            }
        }

        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            map = *msg;
            global_esdf_map.resize(map.info.height*map.info.width);
            updateESDF2d(map,global_esdf_map);
            map_receive_flag = true;
            
        }

        void pos_callback(const geometry_msgs::Point::ConstPtr& msg){
            pos(0) = msg->x;
            pos(1) = msg->y;
            pos(2) = msg->z;
            pos_receive_flag = true;
        }

        void simu_timer_callback(const ros::TimerEvent& event){
            my_local_map.data.clear();
            // my_local_esdf_map.clear();
            if(map_receive_flag && pos_receive_flag){
                double map_origin_x = map.info.origin.position.x;
                double map_origin_y = map.info.origin.position.y;
                double map_resolution = map.info.resolution;
                double start_x = pos(0) - local_map_width/2;
                double start_y = pos(1) - local_map_length/2;
                double grid_x = std::floor((start_x - map_origin_x)/map_resolution);
                double grid_y = std::floor((start_y - map_origin_y)/map_resolution);
                double grid_width = std::floor(local_map_width/map_resolution);
                double grid_length = std::floor(local_map_length/map_resolution);
                for(int i = 0;i < grid_length;++i){
                    int gridy = grid_y + i;
                    for(int j = 0;j < grid_width;++j){                       
                        int gridx = grid_x + j;
                        int idx = gridx + gridy*map.info.width;
                        if(idx >= map.data.size() || idx < 0){
                            ROS_ERROR("THE SIZE OVER THE DATA");                      
                        }
                        else{
                            if(map.data[idx] == 0)
                                my_local_map.data.push_back(0);
                            else
                                my_local_map.data.push_back(1);
                        }                       
                    }
                }
                my_local_map.info.resolution = map_resolution;
                my_local_map.info.height = std::floor(local_map_length/map_resolution);
                my_local_map.info.width = std::floor(local_map_width/map_resolution);
                my_local_map.info.origin.position.x = -local_map_width/2;
                my_local_map.info.origin.position.y = -local_map_length/2;
                my_local_map.info.origin.orientation.w = 1;
                my_local_map.header.frame_id = "/map";
                my_local_map.header.stamp = ros::Time::now();
                my_local_map_pub.publish(my_local_map);
                pos_receive_flag = false;
                my_local_map_flag = true;
                // std::cout<<"agin"<<std::endl;
                updateESDF2d(my_local_map,my_local_esdf_map);
            }
        }

        void find_obs_callback(const ros::TimerEvent& event){
            if(my_local_map_flag){
                std::vector<ConnectedComponent> obs;
                obs = findConnectedComponents();
                // 创建visualization_msgs::MarkerArray消息
                visualization_msgs::MarkerArray marker_array_msg;

                // 循环遍历所有连通域，并将其转换为Marker消息
                for (const auto& component : obs) {
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "map"; // 假设地图的frame_id为"map"，根据实际情况修改
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "connected_components";
                    marker.id = component.id;
                    marker.type = visualization_msgs::Marker::POINTS;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.1; // 栅格点的大小
                    marker.scale.y = 0.1;
                    marker.color.a = 1.0; // 不透明度
                    marker.color.r = 1.0; // 颜色，这里设为红色
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;

                    // 将连通域中的栅格单元转换为点，并添加到Marker消息中
                    for (const auto& cell : component.cells) {
                    int r = cell / my_local_map.info.width;
                    int c = cell % my_local_map.info.width;
                    geometry_msgs::Point point;
                    point.x = c*my_local_map.info.resolution + my_local_map.info.origin.position.x; // 假设栅格地图的原点为(0, 0)
                    point.y = r*my_local_map.info.resolution + my_local_map.info.origin.position.y;
                    marker.points.push_back(point);
                    }

                    marker_array_msg.markers.push_back(marker);
                }

                // 发布MarkerArray消息
                vis_pub.publish(marker_array_msg);
            }
            else{
                ROS_WARN("The local map is not ready!");
            }
            
        }

        // 栅格地图障碍物连通域判断函数
        std::vector<ConnectedComponent> findConnectedComponents() {
            std::vector<std::vector<int>> grid;         
            int rows = my_local_map.info.height;
            int cols = my_local_map.info.width;
            int idx = 0;
            for(int i=0;i<rows;++i){
                std::vector<int> grid_;
                for(int j=0;j<cols;++j){
                    grid_.push_back(my_local_map.data[idx]);
                    idx++;
                }
                grid.push_back(grid_);
            }
            std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
            std::vector<ConnectedComponent> connectedComponents;
            int currentComponentId = 0;

            // 定义8个方向的偏移量，用于DFS
            int dr[] = {-1, -1, -1, 0, 0, 1, 1, 1};
            int dc[] = {-1, 0, 1, -1, 1, -1, 0, 1};

            // 深度优先搜索函数，用于将一个连通域中的所有栅格单元找出来
            std::function<void(int, int, int)> dfs = [&](int r, int c, int componentId) {
                visited[r][c] = true;
                connectedComponents[componentId].cells.push_back(r * cols + c); // 将当前栅格单元加入连通域
                for (int i = 0; i < 8; ++i) {
                int nr = r + dr[i];
                int nc = c + dc[i];
                if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc] && !visited[nr][nc]) {
                    dfs(nr, nc, componentId); // 递归搜索相邻的未访问栅格单元
                }
                }
            };

            // 遍历整个栅格地图，找到所有连通域
            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                if (grid[r][c] && !visited[r][c]) {
                    connectedComponents.push_back(ConnectedComponent()); // 创建新的连通域
                    connectedComponents[currentComponentId].id = currentComponentId;
                    dfs(r, c, currentComponentId); // 使用DFS找到该连通域中的所有栅格单元
                    ++currentComponentId;
                }
                }
            }

            return connectedComponents;
        }

        int find_my_local_map_idx(const std::vector<double>& xy){
            int x = std::floor((xy[0] - pos(0) - my_local_map.info.origin.position.x)/my_local_map.info.resolution);
            int y = std::floor((xy[1] - pos(1) - my_local_map.info.origin.position.y)/my_local_map.info.resolution);
            return x + y*my_local_map.info.width;
        }

        int find_my_local_map_idx_x_y(const double & a,const double& b){
            int x = std::floor((a - pos(0) - my_local_map.info.origin.position.x)/my_local_map.info.resolution);
            int y = std::floor((b - pos(1) - my_local_map.info.origin.position.y)/my_local_map.info.resolution);
            return x + y * my_local_map.info.width;
        }

        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, const nav_msgs::OccupancyGrid& esdf_map,int dim) {
            // int local_map_length_grid = local_map_length/my_local_map.info.resolution;
            // int local_map_width_grid = local_map_width/my_local_map.info.resolution;
            // int esdf_map_length_grid = esdf_map.info.height;
            // int local_map_width_grid = esdf_map.info.width;
            std::vector<unsigned int> esdf_map_grid = {esdf_map.info.width,esdf_map.info.height};
            int v[esdf_map_grid[dim]];
            double z[esdf_map_grid[dim]+1];

            int k = start;
            v[start] = start;
            z[start] = -std::numeric_limits<double>::max();
            z[start + 1] = std::numeric_limits<double>::max();

            for (int q = start + 1; q <= end; q++) {
                k++;
                double s;

                do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
                } while (s <= z[k]);

                k++;

                v[k] = q;
                z[k] = s;
                z[k + 1] = std::numeric_limits<double>::max();
            }

            k = start;

            for (int q = start; q <= end; q++) {
                while (z[k + 1] < q) k++;
                double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
                f_set_val(q, val);
            }
        }   

        void updateESDF2d(const nav_msgs::OccupancyGrid& esdf_map,std::vector<double>& save_esdf_map) {
            // int local_map_length_grid = local_map_length/my_local_map.info.resolution;
            // int local_map_width_grid = local_map_width/my_local_map.info.resolution;
            int esdf_map_length_grid = esdf_map.info.height;
            int esdf_map_width_grid = esdf_map.info.width;
            
            // my_local_map_neg.resize(14161);
            esdf_dis.resize(int(esdf_map_length_grid*esdf_map_width_grid));
            tmp_buffer1.resize(int(esdf_map_length_grid*esdf_map_width_grid));
            // esdf_dis_.resize(int(esdf_map_length_grid*esdf_map_width_grid));
            // esdf_dis.resize(4000000);
            // tmp_buffer1.resize(4000000);
            // esdf_dis_.resize(4000000);
            // esdf_dis_neg.resize(14161);
              
            // tmp_buffer1.clear();
            // esdf_dis.clear();
            // esdf_dis_.clear();   

            /* ========== compute positive DT ========== */
            double t1 = ros::Time::now().toSec();
            for (int x = 0; x < esdf_map_width_grid; x++) {
                fillESDF(
                    [&](double y) {
                        return esdf_map.data[x + y * esdf_map.info.width] != 0 ?
                            0 :
                            std::numeric_limits<double>::max();
                    },
                    [&](double y, double val) {tmp_buffer1[x + y * esdf_map.info.width] = val; }, 0,
                    esdf_map_width_grid , esdf_map,1);
            }

            for (int y = 0; y < esdf_map_length_grid; y++) {
                fillESDF(
                    [&](double x) {
                        return tmp_buffer1[x + y * esdf_map.info.width];},
                    [&](double x, double val) {esdf_dis[x + y * esdf_map.info.width] = esdf_map.info.resolution * std::sqrt(val); }, 0,
                    esdf_map_length_grid , esdf_map,0);
            }

            // for(int x=0;x < local_map_width_grid; ++x){
            //     for(int y = 0;y <= local_map_length_grid; ++y){
            //         int idx = x + y*my_local_map.info.width;
            //         if(my_local_map.data[idx] == 1)
            //             my_local_map_neg[idx] == 0;
            //         else
            //             my_local_map_neg[idx] == 1;
            //     }
            // }

            // // tmp_buffer1.clear();

            // for (int x = 0; x < local_map_width_grid; x++) {
            //     fillESDF(
            //         [&](double y) {
            //             return my_local_map_neg[x + y * my_local_map.info.width] == 1 ?
            //                 0 :
            //                 std::numeric_limits<double>::max();
            //         },
            //         [&](double y, double val) {tmp_buffer1[x + y * my_local_map.info.width] = val; }, 0,
            //         local_map_width_grid , 2);
            // }

            // for (int y = 0; y < local_map_length_grid; y++) {
            //     fillESDF(
            //         [&](double x) {
            //             return tmp_buffer1[x + y * my_local_map.info.width];},
            //         [&](double x, double val) {esdf_dis_neg[std::floor(x + y * my_local_map.info.width)] = my_local_map.info.resolution * std::sqrt(val); }, 0,
            //         local_map_length_grid , 2);
            // }

            // for (int x = 0; x < local_map_width_grid; ++x){
            //     for (int y = 0; y <= local_map_length_grid; ++y){
            //         int idx = x + y*my_local_map.info.width;
            //         esdf_dis_[idx] = esdf_dis[idx];

            //         if (esdf_dis_neg[idx] > 0.0)
            //             esdf_dis_[idx] += (-esdf_dis_neg[idx] + my_local_map.info.resolution);
            //     }
            // }

            for (int x = 0; x < esdf_map_width_grid; ++x){
                for (int y = 0; y <= esdf_map_length_grid; ++y){
                    int idx = x + y*esdf_map.info.width;

                    if (esdf_dis[idx] > 6.0)
                        esdf_dis[idx] = 6;
                    save_esdf_map[idx] = esdf_dis[idx];
                }
            }
            double t2 = ros::Time::now().toSec();
            std::cout << "THE TIME COST : " << (t2 -t1)*1000 << "  ms" <<std::endl;

            double local_map_orix = esdf_map.info.origin.position.x;
            double local_map_oriy = esdf_map.info.origin.position.y;
            int k_idx = 0;
            double min_dist = *(std::min_element(esdf_dis.begin(), esdf_dis.end()));
            double max_dist = *(std::max_element(esdf_dis.begin(), esdf_dis.end()));
            ESDF_point_Array.markers.clear();
            
            for(double x = 0; x< esdf_map_width_grid; x +=10){
                for(double y = 0;y<esdf_map_length_grid;y+=10){
                    double dist;    
                    visualization_msgs::Marker ESDF_point;               
                    dist = esdf_dis[x + y * esdf_map.info.width];
                    geometry_msgs::Point point;
                    point.x = local_map_orix + x * esdf_map.info.resolution;
                    point.y = local_map_oriy + y * esdf_map.info.resolution;
                    ESDF_point.header.frame_id = "map";
                    ESDF_point.header.stamp = ros::Time::now();
                    ESDF_point.ns = "Points";
                    ESDF_point.action = visualization_msgs::Marker::ADD;
                    ESDF_point.pose.orientation.w = 1.0;
                    ESDF_point.id = k_idx; //unique id, useful when multiple markers exist.
                    ESDF_point.type = visualization_msgs::Marker::POINTS; //ESDF_pointer type
                    ESDF_point.scale.x = 0.5; ESDF_point.scale.y = 0.5; //width of the line
                    ESDF_point.color.r = 0; ESDF_point.color.g = 1; ESDF_point.color.b = 0;//color of the line: white.
                    ESDF_point.color.a = (dist - min_dist) / (max_dist - min_dist); //不透明度，设0则全透明
                    ESDF_point.points.push_back(point);
                    ESDF_point_Array.markers.push_back(ESDF_point);
                    k_idx++;
                }
            }
            ESDF_pub.publish(ESDF_point_Array);
        }


};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_local_planner_node");
    my_local_planner mlp;
    
    ros::spin();
    return 0;
}