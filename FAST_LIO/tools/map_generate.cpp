 #include <iostream>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/passthrough.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 #include <pcl/filters/voxel_grid.h>
 #include <string>
 
 class MapGenerate
 {
    public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    MapGenerate();
    void readPcdFile();
    void cloudFilt();
 };

MapGenerate::MapGenerate()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

void MapGenerate::readPcdFile()
{
  std::string file_name = std::string("scans.pcd");
  std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (all_points_dir, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file pcd \n");
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << std::endl;
}

void MapGenerate::cloudFilt()
{
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (0.0, 1.0);
    // pass.filter (*cloud);
    // std::cerr << "Cloud Atfer Z PassThrough " << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> sor_voxel;
    sor_voxel.setInputCloud (cloud);
    sor_voxel.setLeafSize (0.02f, 0.02f, 0.02f);
    sor_voxel.filter (*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
    std::cerr << "Cloud Atfer StatisticalOutlierRemoval " << std::endl;

    std::cout << "Save "
          << cloud->width * cloud->height
          << std::endl;

    pcl::PCDWriter writer;
    std::string file_name = std::string("scans_voxel.pcd");
    std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + file_name);
    writer.write<pcl::PointXYZ> (all_points_dir, *cloud, false);
}

int main(int argc, char **argv)
{
    MapGenerate mg;
    mg.readPcdFile();
    mg.cloudFilt();
}


