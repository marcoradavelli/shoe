#include <iostream>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/time.h>
//#include <obj_recognition/SegmentedClustersArray.h>

ros::Publisher chatter_pub;

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg2 (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("1534596639899838.pcd", *cloud_msg2) == -1) //* load the file
  //{
  //  PCL_ERROR ("Couldn't read file bunny.pcd \n");
  //  return (-1);
  //}

  pcl::PCLPointCloud2 cloud_msg;
  pcl_conversions::toPCL(*msg, cloud_msg);
  //pcl::io::loadPCDFile ("1534596639899838.pcd", cloud_msg);

  //... read, pass in or create a point cloud ...


  // **********    SEGMENTATION PART   ************
// Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
//  pcl_conversions::toPCLPointCloud2(*cloud_msg, *cloud);
//pcl::fromPCLPointCloud2 (cloud_blob, *cloud); //* convert from pcl/PCLPointCloud2 to pcl::PointCloud<T>
  *cloud = cloud_msg;

  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
//  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (*cloudFilteredPtr);


  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  //pass.setFilterLimits (.5, 1.1);
  pass.setFilterLimits (1, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);

  pass.setInputCloud (xyzCloudPtrFiltered);
  pass.setFilterFieldName ("x");
  //pass.setFilterLimits (.5, 1.1);
  pass.setFilterLimits (-0.4, -0.1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);
  
    pass.setInputCloud (xyzCloudPtrFiltered);
  pass.setFilterFieldName ("y");
  //pass.setFilterLimits (.5, 1.1);
  pass.setFilterLimits (-0.12, 0.1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);



  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.02);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);



  // perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  //obj_recognition::SegmentedClustersArray CloudClusters;

  // declare the output variable instances
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

// create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {

    // create a new clusterData message object
    //obj_recognition::ClusterData clusterData;


    
    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);

        }



  }


    // log the position of the cluster
    //clusterData.position[0] = (*cloudPtr).data[0];
    //clusterData.position[1] = (*cloudPtr).points.back().y;
    //clusterData.position[2] = (*cloudPtr).points.back().z;
    //std::string info_string = string(cloudPtr->points.back().x);
    //printf(clusterData.position[0]);

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type  (CAUSES ROS::Time error)
    pcl_conversions::fromPCL(outputPCL, output);

    // add the cluster to the array message
    //clusterData.cluster = output;
    //CloudClusters.clusters.push_back(output);
    
    

  // publish the clusters
  //m_clusterPub.publish(CloudClusters);
  // ***********   END OF SEGMENTATION PART  **********
pcl::PointCloud<pcl::PointXYZRGB>::Ptr input = xyzCloudPtrRansacFiltered;

//pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::copyPointCloud(input,xyzCloudPtrRansacFiltered);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud (input);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (tree2);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*

  //std::cout << "Loaded "
  //          << input->width * input->height
  //          << " data points from test_pcd.pcd with the following fields: "
  //          << std::endl;
  //for (size_t i = 0; i < input->points.size (); ++i)
  //  std::cout << "    " << input->points[i].x
  //            << " "    << input->points[i].y
  //            << " "    << input->points[i].z 
  //            << " "    << cloud_normals->points[i] << std::endl;

 

  //ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
//  int count = 0;
//  while (ros::ok())
//  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //sensor_msgs::PointCloud2 cloud2;
    //pcl::toROSMsg(outputPCL, cloud2); 
    output.header.frame_id = "/myimg";
    output.header.stamp=ros::Time::now();
    //pub.publish (*clusters);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(output);

    ros::spinOnce();

//    loop_rate.sleep();
//    ++count;
//  }
}

int
main (int argc, char** argv)
{


ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

/**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/kinect2/sd/points", 1, chatterCallback);


/**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  //ros::Publisher 
  chatter_pub = n.advertise<sensor_msgs::PointCloud2>("/myimg", 10, true);
  
  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  

  return 0;

}
