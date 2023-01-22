#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <fstream>

class EuclideanClustering{
    private:
        /*node handle*/
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate;
        /*subscribe*/
        ros::Subscriber sub_pc;
        /*publish*/
        ros::Publisher pub_mom;
        ros::Publisher pub_moms;
        ros::Publisher pub_path;
        /*pcl objects*/
        pcl::visualization::PCLVisualizer viewer {"Euclidian Clustering"};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
        pcl::PointCloud<pcl::PointXYZ>::Ptr moms {new pcl::PointCloud<pcl::PointXYZ>};  //出力する重心群
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        geometry_msgs::PointStamped point_msg;
        nav_msgs::Path mom_path;
        /*parameters*/
        double cluster_tolerance;
        int min_cluster_size;

        int target_;
        int cluster_id_;
        double tracking_tolerance;
        double pre_x = 0;
        double pre_y = 0;
        double pre_z = 0;
        bool tracking = false;
        std::string output_csv_path = "~/catkin_ws/src/clustering_pcl/output/output.csv";
        std::string input_csv_path = "~/catkin_ws/src/clustering_pcl/input/input.csv";
    public:
        EuclideanClustering();
        void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
        void Clustering(void);
        void Visualization(void);
};

EuclideanClustering::EuclideanClustering()
    :nhPrivate("~")
{
    sub_pc = nh.subscribe("/velodyne_obstacles", 1, &EuclideanClustering::CallbackPC, this);
    pub_mom = nh.advertise<geometry_msgs::PointStamped>("/moment_point", 10); // テスト用
    pub_moms = nh.advertise<sensor_msgs::PointCloud2>("/moments", 10); // 全クラスターの重心
    pub_path = nh.advertise<nav_msgs::Path>("/mom_path", 10);
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0, "axis");
    viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

    nhPrivate.param("cluster_tolerance", cluster_tolerance, 5.0);
    nhPrivate.param("min_cluster_size", min_cluster_size, 30);
    nhPrivate.param("target_", target_, 1);
    nhPrivate.param("cluster_id_", cluster_id_, 1);
    nhPrivate.param("tracking_tolerance", tracking_tolerance, 5.0);
    std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
    std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void EuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* std::cout << "CALLBACK PC" << std::endl; */

    pcl::fromROSMsg(*msg, *cloud);
    std::cout << "==========" << std::endl;
    std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

    moms->width = 100;
    moms->height = 1;
    moms->points.resize(moms->width * moms->height);
    std::cout << "moms ->points.size() = " << moms->points.size() << std::endl;

    clusters.clear();
    Clustering();
    Visualization();
}

void EuclideanClustering::Clustering(void)
{
    double time_start = ros::Time::now().toSec();

    /*clustering*/
    /*kd-treeクラスを宣言*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    /*探索する点群をinput*/
    tree->setInputCloud(cloud);
    /*クラスタリング後のインデックスが格納されるベクトル*/
    std::vector<pcl::PointIndices> cluster_indices;
    /*今回の主役*/
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    /*距離の閾値を設定*/
    ece.setClusterTolerance(cluster_tolerance);
    // std::cout << "---cluster_tolerance=" << cluster_tolerance << "---" << std::endl;
    /*各クラスタのメンバの最小数を設定*/
    ece.setMinClusterSize(min_cluster_size);
    /*各クラスタのメンバの最大数を設定*/
    ece.setMaxClusterSize(cloud->points.size());
    /*探索方法を設定*/
    ece.setSearchMethod(tree);
    /*クラスリング対象の点群をinput*/
    ece.setInputCloud(cloud);
    /*クラスリング実行*/
    ece.extract(cluster_indices);

    std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

    std::cout << "target object" << std::endl;
    std::cout << " pre_x: " << pre_x << std::endl;
    std::cout << " pre_y: " << pre_y << std::endl;
    std::cout << " pre_z: " << pre_z << std::endl;

    /*dividing（クラスタごとに点群を分割)*/
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    ei.setInputCloud(cloud);
    ei.setNegative(false);
    for(size_t i=0;i<cluster_indices.size();i++){
        /*extract*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
        *tmp_clustered_indices = cluster_indices[i];
        ei.setIndices(tmp_clustered_indices);
        ei.filter(*tmp_clustered_points);
        /*input*/
        clusters.push_back(tmp_clustered_points);
    }

    /* クラスタの重心の計算 */
    int i = 0; // cluster count
    double dist = 0; // distance
    double dx, dy, dz;
    bool addPath = false; // pathが追加されたか
    std::ofstream ofs_csv_file(output_csv_path);
    /* 全クラスタについて重心を計算 */
    for(auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it){ // cluster size
        double x_ave = 0;
        double y_ave = 0;
        double z_ave = 0;
        int pt_size = cluster_indices[i].indices.size();

        for(auto pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            double x = cloud->points[*pit].x;
            double y = cloud->points[*pit].y;
            double z = cloud->points[*pit].z;

            x_ave += x;
            y_ave += y;
            z_ave += z;
        }

        x_ave = x_ave / pt_size;
        y_ave = y_ave / pt_size;
        z_ave = z_ave / pt_size;

        // 重心の処理 //
        std::cout << "pt_size: " << pt_size << std::endl;
        std::cout << " i    :  " << i << std::endl;
        std::cout << " x_ave:  " << x_ave << std::endl;
        std::cout << " y_ave:  " << y_ave << std::endl;
        std::cout << " z_ave:  " << z_ave << std::endl;

        // point cloud
        moms->points[i].x = x_ave;
        moms->points[i].y = y_ave;
        moms->points[i].z = z_ave;

        /* 出力する軌跡の作成 */
        // 追跡したいクラスターの設定
        if(i==cluster_id_ && tracking==false){
            pre_x = x_ave;
            pre_y = y_ave;
            pre_z = z_ave;
            tracking = true;
            mom_path.poses.clear();
            std::cout << "---start tracking---" << std::endl;
            std::cout << " x_ave:  " << x_ave << std::endl;
            std::cout << " y_ave:  " << y_ave << std::endl;
            std::cout << " z_ave:  " << z_ave << std::endl;
        }
        // それぞれの重心と追跡したい重心の距離を求める
        dx = x_ave - pre_x;
        dy = y_ave - pre_y;
        dz = z_ave - pre_z;
        dist = sqrt( pow(dx,2)+pow(dy,2)+pow(dz,2) );
        std::cout << " dist :  " << dist << std::endl;

        // 重心が追跡したい物体と近ければpathに追加
        if(dist <= tracking_tolerance){
            std::cout << " dx:     " << dx << std::endl;
            std::cout << " dy:     " << dy << std::endl;
            std::cout << " dx:     " << dz << std::endl;
            // 一定距離以内でpathに追加
            std::cout << "  dist:   " << dist << std::endl;
            if(dist <= tracking_tolerance && !addPath)
            {
                std::cout << "  ^ add path---" << std::endl;
                addPath = true;
                geometry_msgs::PoseStamped path_point;
                path_point.pose.position.x = x_ave;
                path_point.pose.position.y = y_ave;
                path_point.pose.position.z = z_ave;
                path_point.pose.orientation.w = 1;
                mom_path.poses.push_back(path_point);
            }
            // 値保存
            pre_x = x_ave;
            pre_y = y_ave;
            pre_z = z_ave;
        }

        i++; // next index

    }
    // 全クラスタの探索終了

    // 一定距離内の点がなかったらpathをクリア&出力
    if(!addPath)
    {
        std::cout << "---cleared path---" << std::endl;
        mom_path.poses.clear();
        tracking = false;
    }

    // publishのあれこれ
    sensor_msgs::PointCloud2 mom_pc;
    pcl::toROSMsg(*moms, mom_pc);

    mom_pc.header.frame_id = "velodyne";
    mom_pc.header.stamp = ros::Time::now();
    mom_path.header.frame_id = "velodyne";
    mom_path.header.stamp = ros::Time::now();

    pub_moms.publish(mom_pc);
    pub_path.publish(mom_path);

    moms->points.clear();

    std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void EuclideanClustering::Visualization(void)
{
    viewer.removeAllPointClouds();

    /*cloud*/
    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    /*clusters*/
    double rgb[3] = {};
    const int channel = 3;
    const double step = ceil(pow(clusters.size()+2, 1.0/(double)channel));	//exept (000),(111)
    const double max = 1.0;
    // std::cout << "cl.size:" << clusters.size() << std::endl;
    for(size_t i=0;i<clusters.size();i++){
        std::string name = "cluster_" + std::to_string(i);
        rgb[0] += 1/step;
        for(int j=0;j<channel-1;j++){
            if(rgb[j]>max){
                rgb[j] -= max + 1/step;
                rgb[j+1] += 1/step;
            }
        }
        // std::cout << "rgb[0]:" << rgb[0] << std::endl;
        viewer.addPointCloud(clusters[i], name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }

    viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");

    EuclideanClustering euclidean_clustering;

    ros::spin();
}
