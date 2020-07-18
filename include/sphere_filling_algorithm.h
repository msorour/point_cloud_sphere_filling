
#ifndef __GRASPING_ALGORITHM_H__
#define __GRASPING_ALGORITHM_H__

#include <pcl/surface/concave_hull.h>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>


void point_cloud_as_set_of_spheres_fixed_radius( pcl::PointCloud<pcl::PointXYZ>::Ptr&  input_point_cloud_xyz,    // input
                                                 double sphere_radius,                                           // input
                                                 double overlap_distance,                                        // input
                                                 int iterations,                                                 // input
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr&  spheres_filtered_xyz,     // output
                                                 Eigen::Vector4f& far_point_in_pos_direction_4d,                 // output
                                                 Eigen::Vector4f& far_point_in_neg_direction_4d ){               // output
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr   sphere_offset_cloud_xyz   (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr   dummy_cloud_xyz           (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr   poly_mesh_vertices        (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::PolygonMesh mesh_out;
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull;
  
  double value_x, value_y, value_z;
  pcl::PointXYZ point_xyz;
  double sphere_value;
  
  // centroid of workspace point cloud
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> input_cloud_centroid;
  pcl::PointXYZ input_cloud_centroid_point;
  for(int i=0; i<input_point_cloud_xyz->points.size(); i++)
    input_cloud_centroid.add( input_point_cloud_xyz->points[i] );
  input_cloud_centroid.get(input_cloud_centroid_point);
  
  // get the far point in negative and positive directions
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = input_cloud_centroid_point;
  far_point_in_neg_direction = input_cloud_centroid_point;
  for(unsigned int i=0;i<input_point_cloud_xyz->size();i++){
    if( input_point_cloud_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = input_point_cloud_xyz->points[i].x;
    if( input_point_cloud_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = input_point_cloud_xyz->points[i].x;
    
    if( input_point_cloud_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = input_point_cloud_xyz->points[i].y;
    if( input_point_cloud_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = input_point_cloud_xyz->points[i].y;
    
    if( input_point_cloud_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = input_point_cloud_xyz->points[i].z;
    if( input_point_cloud_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = input_point_cloud_xyz->points[i].z;
  }
  far_point_in_pos_direction_4d << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_4d << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  pcl::PointXYZ input_cloud_lattice_dimensions;
  input_cloud_lattice_dimensions.x = (far_point_in_pos_direction.x - far_point_in_neg_direction.x);
  input_cloud_lattice_dimensions.y = (far_point_in_pos_direction.y - far_point_in_neg_direction.y);
  input_cloud_lattice_dimensions.z = (far_point_in_pos_direction.z - far_point_in_neg_direction.z);
  
  // generating the concave hull
  concave_hull.setInputCloud( input_point_cloud_xyz );
  concave_hull.setAlpha( 0.01f );
  concave_hull.reconstruct( mesh_out );
  
  // getting the vertices of the polygon mesh generated
  pcl::fromPCLPointCloud2(mesh_out.cloud, *poly_mesh_vertices);
  
  // shifting in x, y, and z-axes to find the biggest fitting spheres
  for(int l=0; l<iterations; l++){
    for(int m=0; m<iterations; m++){
      for(int n=0; n<iterations; n++){
        // construct the sphere lattice
        sphere_offset_cloud_xyz->clear();
        int number_of_spheres_in_x = input_cloud_lattice_dimensions.x/(2*sphere_radius) +1;
        int number_of_spheres_in_y = input_cloud_lattice_dimensions.y/(2*sphere_radius) +1;
        int number_of_spheres_in_z = input_cloud_lattice_dimensions.z/(2*sphere_radius) +1;
        for(int i=0; i<number_of_spheres_in_x; i++){
          for(int j=0; j<number_of_spheres_in_y; j++){
            for(int k=0; k<number_of_spheres_in_z; k++){
              point_xyz.x = far_point_in_neg_direction.x+sphere_radius*(l+1)/iterations + i*2*sphere_radius;
              point_xyz.y = far_point_in_neg_direction.y+sphere_radius*(m+1)/iterations + j*2*sphere_radius;
              point_xyz.z = far_point_in_neg_direction.z+sphere_radius*(n+1)/iterations + k*2*sphere_radius;
              sphere_offset_cloud_xyz->points.push_back( point_xyz );
            }
          }
        }
        
        // filtering the sphere lattice (#1)
        spheres_filtered_xyz->clear();
        for(int j=0; j<sphere_offset_cloud_xyz->size(); j++){
          for(int k=0; k<input_point_cloud_xyz->size(); k++){
            sphere_value = ( pow(input_point_cloud_xyz->points[k].x - sphere_offset_cloud_xyz->points[j].x, 2)
                           + pow(input_point_cloud_xyz->points[k].y - sphere_offset_cloud_xyz->points[j].y, 2) 
                           + pow(input_point_cloud_xyz->points[k].z - sphere_offset_cloud_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value <= 1){
              spheres_filtered_xyz->points.push_back( sphere_offset_cloud_xyz->points[j] );
              break;
            }
          }
        }
        
        // filtering the sphere lattice (#2)
        //dummy_cloud_xyz->clear();
        int number_of_mesh_points_inside_this_sphere;
        for(int j=0; j<spheres_filtered_xyz->size(); j++){
          number_of_mesh_points_inside_this_sphere = 0;
          for(int k=0; k<poly_mesh_vertices->size(); k++){
            sphere_value = ( pow(poly_mesh_vertices->points[k].x - spheres_filtered_xyz->points[j].x, 2)
                           + pow(poly_mesh_vertices->points[k].y - spheres_filtered_xyz->points[j].y, 2) 
                           + pow(poly_mesh_vertices->points[k].z - spheres_filtered_xyz->points[j].z, 2) )/pow(sphere_radius, 2);
            if(sphere_value < 1)
              number_of_mesh_points_inside_this_sphere++;
          }
          if(number_of_mesh_points_inside_this_sphere == 0)
            dummy_cloud_xyz->points.push_back( spheres_filtered_xyz->points[j] );
        }
        
      }
    }
  }
  *spheres_filtered_xyz = *dummy_cloud_xyz;
  
  // filtering (#3)
  // having all these spheres, we check the center of each sphere if it exists in another fellow sphere
  // if it does we remove it
  std::vector<double> sphere_offset_vector_x, sphere_offset_vector_y, sphere_offset_vector_z;
  for(int i=0; i<spheres_filtered_xyz->size(); i++){
    sphere_offset_vector_x.push_back( spheres_filtered_xyz->points[i].x );
    sphere_offset_vector_y.push_back( spheres_filtered_xyz->points[i].y );
    sphere_offset_vector_z.push_back( spheres_filtered_xyz->points[i].z );
  }
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    for(int j=sphere_offset_vector_x.size()-1; j>-1; j--){
      sphere_value = ( pow(sphere_offset_vector_x[j] - sphere_offset_vector_x[i], 2)
                     + pow(sphere_offset_vector_y[j] - sphere_offset_vector_y[i], 2) 
                     + pow(sphere_offset_vector_z[j] - sphere_offset_vector_z[i], 2) )/pow(sphere_radius, 2);
      // the center of this sphere is on the border of another sphere
      if(sphere_value < overlap_distance and i!=j){
        sphere_offset_vector_x.erase(sphere_offset_vector_x.begin()+j);
        sphere_offset_vector_y.erase(sphere_offset_vector_y.begin()+j);
        sphere_offset_vector_z.erase(sphere_offset_vector_z.begin()+j);
      }
    }
  }
  
  spheres_filtered_xyz->clear();
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    point_xyz.x = sphere_offset_vector_x[i];
    point_xyz.y = sphere_offset_vector_y[i];
    point_xyz.z = sphere_offset_vector_z[i];
    spheres_filtered_xyz->points.push_back( point_xyz );
  }
  
  /*
  // get the index with highest number of spheres
  std::vector<int> number_of_filling_spheres;
  number_of_filling_spheres.push_back( spheres_filtered_xyz->size() );
  std::vector<int>::iterator it = std::find(number_of_filling_spheres.begin(), number_of_filling_spheres.end(), *std::max_element(number_of_filling_spheres.begin(), number_of_filling_spheres.end()) );
  int index_of_greatest = std::distance(number_of_filling_spheres.begin(), it);
  //std::cout <<  "index = " << index_of_greatest << std::endl;
  //std::cout <<  "value = " << number_of_filling_spheres[index_of_greatest] << std::endl;
  */
  
  
}

void point_cloud_as_set_of_spheres( int desired_number_of_spheres,                                      // input
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  input_point_cloud_xyz,        // input
                                    double radius_of_smallest_sphere,                                   // input
                                    double radius_of_largest_sphere,                                    // input
                                    double sphere_radius_increment,                                     // input
                                    double overlap_distance,                                            // input
                                    int iterations,                                                     // input
                                    int sphere_point_cloud_samples,                                     // input (this is the number of points to use in drawing each sphere point cloud in visualization)
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  spheres_filtered_xyz,         // output
                                    std::vector<double>& sphere_radius_filtered,                        // output
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr&  spheres_visualization_xyz,    // output
                                    Eigen::Vector4f& far_point_in_pos_direction_4d,                     // output
                                    Eigen::Vector4f& far_point_in_neg_direction_4d ){                   // output
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr  dummy_cloud_xyz  (new pcl::PointCloud<pcl::PointXYZ>);
  double value_x, value_y, value_z;
  pcl::PointXYZ point_xyz;
  double sphere_value;
  
  // centroid of workspace point cloud
  // object plane centroid location
  pcl::CentroidPoint<pcl::PointXYZ> input_cloud_centroid;
  pcl::PointXYZ input_cloud_centroid_point;
  for(int i=0; i<input_point_cloud_xyz->points.size(); i++)
    input_cloud_centroid.add( input_point_cloud_xyz->points[i] );
  input_cloud_centroid.get(input_cloud_centroid_point);
  
  // get the far point in negative and positive directions
  pcl::PointXYZ far_point_in_pos_direction;
  pcl::PointXYZ far_point_in_neg_direction;
  far_point_in_pos_direction = input_cloud_centroid_point;
  far_point_in_neg_direction = input_cloud_centroid_point;
  for(unsigned int i=0;i<input_point_cloud_xyz->size();i++){
    if( input_point_cloud_xyz->points[i].x > far_point_in_pos_direction.x )
      far_point_in_pos_direction.x = input_point_cloud_xyz->points[i].x;
    if( input_point_cloud_xyz->points[i].x < far_point_in_neg_direction.x )
      far_point_in_neg_direction.x = input_point_cloud_xyz->points[i].x;
    
    if( input_point_cloud_xyz->points[i].y > far_point_in_pos_direction.y )
      far_point_in_pos_direction.y = input_point_cloud_xyz->points[i].y;
    if( input_point_cloud_xyz->points[i].y < far_point_in_neg_direction.y )
      far_point_in_neg_direction.y = input_point_cloud_xyz->points[i].y;
    
    if( input_point_cloud_xyz->points[i].z > far_point_in_pos_direction.z )
      far_point_in_pos_direction.z = input_point_cloud_xyz->points[i].z;
    if( input_point_cloud_xyz->points[i].z < far_point_in_neg_direction.z )
      far_point_in_neg_direction.z = input_point_cloud_xyz->points[i].z;
  }
  far_point_in_pos_direction_4d << far_point_in_pos_direction.x, far_point_in_pos_direction.y, far_point_in_pos_direction.z, 1;
  far_point_in_neg_direction_4d << far_point_in_neg_direction.x, far_point_in_neg_direction.y, far_point_in_neg_direction.z, 1;
  
  pcl::PointXYZ input_cloud_lattice_dimensions;
  input_cloud_lattice_dimensions.x = (far_point_in_pos_direction.x - far_point_in_neg_direction.x);
  input_cloud_lattice_dimensions.y = (far_point_in_pos_direction.y - far_point_in_neg_direction.y);
  input_cloud_lattice_dimensions.z = (far_point_in_pos_direction.z - far_point_in_neg_direction.z);
  
  
  //
  std::vector<double> sphere_radius;
  std::vector<double> sphere_radius_dummy;
  //std::vector<double> sphere_radius_filtered;
  int number_of_increments = ceil((radius_of_largest_sphere - radius_of_smallest_sphere)/sphere_radius_increment);
	if(number_of_increments==0)
		number_of_increments+=1;
  for(int i=0; i<number_of_increments; i++)
    sphere_radius.push_back( radius_of_largest_sphere-i*sphere_radius_increment );
  
  for(int i=0; i<sphere_radius.size(); i++){
    point_cloud_as_set_of_spheres_fixed_radius( input_point_cloud_xyz, sphere_radius[i], overlap_distance, iterations, dummy_cloud_xyz, far_point_in_pos_direction_4d, far_point_in_neg_direction_4d );
    std::cout << "radius = " << sphere_radius[i] << ", number of spheres found = " <<  dummy_cloud_xyz->size() << std::endl;
    *spheres_filtered_xyz += *dummy_cloud_xyz;
    for(int j=0; j<dummy_cloud_xyz->size(); j++)
      sphere_radius_dummy.push_back(sphere_radius[i]);
  }
  
  // filtering
  // having all these spheres, we check the center of each sphere if it exists in another fellow sphere
  // if it does we remove it, we give priority to bigger spheres
  std::vector<double> sphere_offset_vector_x, sphere_offset_vector_y, sphere_offset_vector_z;
  for(int i=0; i<spheres_filtered_xyz->size(); i++){
    sphere_offset_vector_x.push_back( spheres_filtered_xyz->points[i].x );
    sphere_offset_vector_y.push_back( spheres_filtered_xyz->points[i].y );
    sphere_offset_vector_z.push_back( spheres_filtered_xyz->points[i].z );
  }
  for(int i=0; i<sphere_offset_vector_x.size(); i++){
    for(int j=sphere_offset_vector_x.size()-1; j>-1; j--){
      sphere_value = ( pow(sphere_offset_vector_x[j] - sphere_offset_vector_x[i], 2)
                     + pow(sphere_offset_vector_y[j] - sphere_offset_vector_y[i], 2) 
                     + pow(sphere_offset_vector_z[j] - sphere_offset_vector_z[i], 2) )/pow(sphere_radius_dummy[i], 2);
      // the center of this sphere is on the border of another sphere
      if(sphere_value < overlap_distance and i!=j){
        sphere_offset_vector_x.erase(sphere_offset_vector_x.begin()+j);
        sphere_offset_vector_y.erase(sphere_offset_vector_y.begin()+j);
        sphere_offset_vector_z.erase(sphere_offset_vector_z.begin()+j);
        sphere_radius_dummy.erase(sphere_radius_dummy.begin()+j);
      }
    }
  }
  
  //
  spheres_filtered_xyz->clear();
  if( desired_number_of_spheres < sphere_offset_vector_x.size() )
    for(int i=0; i<desired_number_of_spheres; i++){
      point_xyz.x = sphere_offset_vector_x[i];
      point_xyz.y = sphere_offset_vector_y[i];
      point_xyz.z = sphere_offset_vector_z[i];
      spheres_filtered_xyz->points.push_back( point_xyz );
      sphere_radius_filtered.push_back( sphere_radius_dummy[i] );
    }
  else{
    for(int i=0; i<sphere_offset_vector_x.size(); i++){
      point_xyz.x = sphere_offset_vector_x[i];
      point_xyz.y = sphere_offset_vector_y[i];
      point_xyz.z = sphere_offset_vector_z[i];
      spheres_filtered_xyz->points.push_back( point_xyz );
      sphere_radius_filtered.push_back( sphere_radius_dummy[i] );
    }
  }
  
  // generate and view the point cloud of the sphere
  for(int j=0; j<spheres_filtered_xyz->size(); j++){
    for(int k=0; k<sphere_point_cloud_samples; k++){
      value_x = (-sphere_radius_filtered[j]+spheres_filtered_xyz->points[j].x) + k*2*sphere_radius_filtered[j]/sphere_point_cloud_samples;
      for(int l=0; l<sphere_point_cloud_samples; l++){
        value_y = (-sphere_radius_filtered[j]+spheres_filtered_xyz->points[j].y) + l*2*sphere_radius_filtered[j]/sphere_point_cloud_samples;
        value_z = spheres_filtered_xyz->points[j].z + sphere_radius_filtered[j]*sqrt( 1 - pow(value_x-spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius_filtered[j], 2) - pow(value_y-spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius_filtered[j], 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        spheres_visualization_xyz->points.push_back( point_xyz );
        
        value_z = spheres_filtered_xyz->points[j].z - sphere_radius_filtered[j]*sqrt( 1 - pow(value_x-spheres_filtered_xyz->points[j].x, 2)/pow(sphere_radius_filtered[j], 2) - pow(value_y-spheres_filtered_xyz->points[j].y, 2)/pow(sphere_radius_filtered[j], 2) );
        point_xyz.x = value_x;
        point_xyz.y = value_y;
        point_xyz.z = value_z;
        
        spheres_visualization_xyz->points.push_back( point_xyz );          
      }
    }
  }
  
}



#endif



