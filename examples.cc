#include <stdio.h>
#include <fstream>
#include <iostream>

#include "vacancy/voxel_carver.h"



std::vector<Eigen::Affine3d> read_camera_pose(std::string filePath)
{

  std::vector<Eigen::Affine3d> res;

  FILE* fp = fopen(filePath.c_str(),"r");

  float tmp[16];


  while (~fscanf(fp,"%f",&tmp[0]))
  {
    for (int i=1;i<12;++i)
      fscanf(fp,"%f",&tmp[i]);

    double T_a[17];

    
    for (int i=0;i<3;++i)
      T_a[0+i*4] = tmp[3+i];
    
    for (int i=0;i<3;++i)
      T_a[1+i*4] = tmp[6+i];

    for (int i=0;i<3;++i)
      T_a[2+i*4] = tmp[0+i];

    for (int i=0;i<3;++i)
      T_a[3+i*4] = tmp[9+i];

    for (int i=0;i<4;++i)
      T_a[i+12] = 0.0;
    
    T_a[15] = 1.0;

    Eigen::Matrix4d pose(T_a);  
    pose = pose.transpose().eval();

    Eigen::Affine3d tt;
    tt= pose;  
    //std::cout<<pose<<std::endl;
    res.push_back(tt);
    //printf("%d\n",res.size());
  }

  printf("Read %d camera poses.\n",res.size());

  fclose(fp);

  return res;
}

std::vector<vacancy::Image1b> load_silhouette(std::string filePath,int N)
{
  std::vector<vacancy::Image1b> res;

  for (int i=0;i<N;++i)
  {
    char filename[64];
    sprintf(filename,"img_%04d.jpg",i);

    vacancy::Image3b mask_img;
    mask_img.Load(filePath +std::string(filename) );


    vacancy::Image1b res_mask;
    vacancy::RGB2Mask(mask_img, &res_mask);
    res.push_back(res_mask);

    //printf("%d %d\n",res_mask.width(),res_mask.height());

  }

  return res;
}


std::vector<std::shared_ptr<vacancy::Camera>> read_camera_intrinsic(std::string filePath, const std::vector<vacancy::Image1b>& silhouettes)
{
  std::vector<std::shared_ptr<vacancy::Camera>> res;
  FILE* fp = fopen(filePath.c_str(),"r");

  int no;

  while (~fscanf(fp,"%d",&no))
  {
    float tmp[9];
    for (int i=0;i<9;++i)
      fscanf(fp,"%f",&tmp[i]);


    int width = silhouettes[no].width();
    int height = silhouettes[no].height();
    Eigen::Vector2f principal_point(tmp[2],tmp[5]);
    Eigen::Vector2f focal_length(tmp[0],tmp[4]);
    std::shared_ptr<vacancy::Camera> camera =
        std::make_shared<vacancy::PinholeCamera>(width, height,
                                                Eigen::Affine3d::Identity(),
                                                principal_point, focal_length);

    //printf("%f %f.\n",tmp[2],tmp[5]);
    res.push_back(camera);
      
  }
  fclose(fp);
  return res;

}



// test by bunny data with 6 views
int main(int argc, char *argv[]) 
{
  if (argc<2)
  {
    printf("Error args!!.\n");
    printf("-----------------------------------\n");
    printf("1.Path to CamPose.inf\n");
    printf("2.Path to Intrinsic.inf\n");
    printf("3.Path to mask folder\n");
    printf("4.Path to output folder\n");
    printf("5.6.7 bb_min_x bb_min_y bb_min_z\n");
    printf("8,9,10 bb_max_x bb_max_y bb_max_z\n");
    printf("11. bb_offset\n");
    exit(-1);
  }





  std::string data_dir{argv[4]};


  auto cam_pos = read_camera_pose(argv[1]);

  auto silhouettes = load_silhouette(argv[3],cam_pos.size());

  auto cam_K = read_camera_intrinsic(argv[2],silhouettes);

/*
  auto cam_pos = read_camera_pose("/data/wmy/NR/dataset/sport_1_mask/CamPose.inf");

  auto silhouettes = load_silhouette("/data/wmy/NR/dataset/sport_1_mask/img/0/mask/",cam_pos.size());

  auto cam_K = read_camera_intrinsic("/data/wmy/NR/dataset/sport_1_mask/Intrinsic.inf",silhouettes);
*/
  



  vacancy::VoxelCarver carver;
  vacancy::VoxelCarverOption option;


  float a,b,c;

  sscanf(argv[5],"%f",&a);
  sscanf(argv[6],"%f",&b);
  sscanf(argv[7],"%f",&c);

  // exact mesh bounding box computed in advacne
  option.bb_min = Eigen::Vector3f(a,b,c);

  sscanf(argv[8],"%f",&a);
  sscanf(argv[9],"%f",&b);
  sscanf(argv[10],"%f",&c);


  option.bb_max = Eigen::Vector3f(a,b,c);

  //option.bb_min = Eigen::Vector3f(-1.0f, -1.5f, -5.0f);
  //option.bb_max = Eigen::Vector3f(1.1f, 0.0f, -3.3f);
  // add offset to the bounding box to keep boundary clean
  float bb_offset = 0.2f;

  sscanf(argv[11],"%f",&bb_offset);

  option.bb_min[0] -= bb_offset;
  option.bb_min[1] -= bb_offset;
  option.bb_min[2] -= bb_offset;

  option.bb_max[0] += bb_offset;
  option.bb_max[1] += bb_offset;
  option.bb_max[2] += bb_offset;

  // voxel resolution is 10mm
  option.resolution = 0.01f;

  carver.set_option(option);

  carver.Init();

  printf("===============BEGIN =======================\n");

  for (size_t i = 0; i < (int)cam_pos.size(); i++) {
    std::shared_ptr<vacancy::Camera> camera = cam_K[i];
    camera->set_c2w(cam_pos[i]);

    std::string num = vacancy::zfill(i);


    vacancy::Image1b silhouette = silhouettes[i];
    

    vacancy::Image1f sdf;
    // Carve() is the main process to update voxels. Corresponds to the fusion
    // step in KinectFusion
    carver.Carve(*camera, silhouette, &sdf);

    // save SDF visualization
    vacancy::Image3b vis_sdf;
    vacancy::SignedDistance2Color(sdf, &vis_sdf, -1.0f, 1.0f);
    vis_sdf.WritePng(data_dir + "/sdf_" + num + ".png");
    

    vacancy::Mesh mesh;
    // voxel extraction
    // slow for algorithm itself and saving to disk
    //carver.ExtractVoxel(&mesh);
    //mesh.WritePly(data_dir + "/voxel_" + num + ".ply");
    // marching cubes
    // smoother and faster
    carver.ExtractIsoSurface(&mesh, 0.0);
    mesh.WritePly(data_dir + "/surface_" + num + ".ply");
  }

  return 0;
}
