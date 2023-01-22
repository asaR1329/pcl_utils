#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

class EuclideanClustering{
    private:
        /*node handlee*/
        ros::NodeHandle nh;
        ros::NodeHandle nhPrivate;
        /*subscribe*/
        ros::Subscriber sub_pc;
        /*pcl objects*/
        pcl::visualization::PCLVisualizer viewer {"Euclidian Clustering"};
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        /*parameters*/
        double cluster_tolerance;
        int min_cluster_size;
    public:
        EuclideanClustering();
        void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
        void Clustering(void);
        void Visualization(void);
};

EuclideanClustering::EuclideanClustering()
    :nhPrivate("~")
{
    sub_pc = nh.subscribe("/velodyne_points", 1, &EuclideanClustering::CallbackPC, this);
    viewer.setBackgroundColor(1, 1, 1);
    viewer.addCoordinateSystem(1.0, "axis");
    viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

    nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.7);
    nhPrivate.param("min_cluster_size", min_cluster_size, 50);
    std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
    std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void EuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    /* std::cout << "CALLBACK PC" << std::endl; */

    pcl::fromROSMsg(*msg, *cloud);
    std::cout << "==========" << std::endl;
    std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

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

    /*dividing（クラスタごとに点群を分割）*/
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

    std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void EuclideanClustering::Visualization(void)
{
    /*前ステップの可視化をリセット*/
    viewer.removeAllPointClouds();

    /*cloud*/
    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    /*clusters*/
    double rgb[3] = {};
    const int channel = 3;	//RGB
    const double step = ceil(pow(clusters.size()+2, 1.0/(double)channel));	//exept (000),(111)
    const double max = 1.0;
    /*クラスタをいい感じに色分け*/
    for(size_t i=0;i<clusters.size();i++){
        std::string name = "cluster_" + std::to_string(i);
        rgb[0] += 1/step;
        for(int j=0;j<channel-1;j++){
            if(rgb[j]>max){
                rgb[j] -= max + 1/step;
                rgb[j+1] += 1/step;
            }
        }
        viewer.addPointCloud(clusters[i], name);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }
    /*表示の更新*/
    viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");

    EuclideanClustering euclidean_clustering;

    ros::spin();
}

