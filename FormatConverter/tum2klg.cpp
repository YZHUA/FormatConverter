#include <iostream>
#include <OpenNI.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <string.h>
#include <kdtree.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv ;
using namespace std;
typedef struct
{
    vector<double>time_stamps;
    vector<string>Depth_files,color_files;
}list;
void loadList(char *listFile,list &a)
{
    FILE *fp=fopen(listFile,"r");
    do{
        double time_stamp;char depthFile[400],colorFile[400];
        fscanf(fp,"%lf %s %s",&time_stamp,depthFile,colorFile);
        a.time_stamps.push_back(time_stamp);
        a.Depth_files.push_back(depthFile);
        a.color_files.push_back(colorFile);
    }while(!feof(fp));
    fclose(fp);
}
kdtree::KDTree* buildtree(vector<long double>data)
{
        kdtree::KDTree*     M_tree;
        kdtree::KDTreeArray M_data;
        int M_num = data.size();
        M_data.resize(boost::extents[M_num][1]);
        for (int m = 0; m<M_num; m++)
            for (int n = 0; n<1; n++)
                M_data[m][n] = (long double)data[m + n];
        // build a kd tree from the model point cloud
        M_tree = new kdtree::KDTree(M_data);
        return M_tree;
}
void mergeList(char *depthFile,char *colorFile,
               char *GroundTruthFile,char *trajectoryFile,char *listFile){
    vector<long double>depth_time_stamps,color_time_stamps,gr_time_stamps;
    vector<string>depthFiles,colorFiles;vector<double>grs;
    //1.读取文件
    long double time_stamp;
    //depth
    FILE *fp=fopen(depthFile,"r");
    char str_tmp[1000];
    for(int i=0;i<4;++i)
        fscanf(fp,"%[^\n]\n",str_tmp);//前面四行没用
    do{
        fscanf(fp,"%llf %s",&time_stamp,str_tmp);
        depth_time_stamps.push_back(time_stamp);
        depthFiles.push_back(str_tmp);
    }while(!feof(fp));
    fclose(fp);
    //rgb
    fp=fopen(colorFile,"r");
    for(int i=0;i<4;++i)
        fscanf(fp,"%[^\n]\n",str_tmp);//前面四行没用
    do{
        fscanf(fp,"%llf %s",&time_stamp,str_tmp);
        color_time_stamps.push_back(time_stamp);
        colorFiles.push_back(str_tmp);
    }while(!feof(fp));
    fclose(fp);
    //gr
    fp=fopen(GroundTruthFile,"r");
    for(int i=0;i<4;++i)
        fscanf(fp,"%[^\n]\n",str_tmp);//前面四行没用
    do{
        double tx,ty,tz,qx,qy,qz,qw;
        fscanf(fp,"%llf %lf %lf %lf %lf %lf %lf %lf",&time_stamp,
               &tx,&ty,&tz,&qx,&qy,&qz,&qw);
        gr_time_stamps.push_back(time_stamp);
        grs.push_back(tx);grs.push_back(ty);grs.push_back(tz);grs.push_back(qx);
        grs.push_back(qy);grs.push_back(qz);grs.push_back(qw);
    }while(!feof(fp));
    fclose(fp);
    //2.找到三者一一对应的时间序列
    //2.1建立kdtree
    kdtree::KDTree* tree_rgb=buildtree(depth_time_stamps);
    kdtree::KDTree* tree_gr=buildtree(gr_time_stamps);
    vector<long double>query(1);kdtree::KDTreeResultVector Argb,Agr;
    //3.保存一致性结果
    FILE *fp_l=fopen(listFile,"w");
    FILE *fp_t=fopen(trajectoryFile,"w");
    for(int i=0;i<depth_time_stamps.size();++i){
        query[0]=depth_time_stamps[i];
        tree_rgb->n_nearest(query,1,Argb);
        tree_gr->n_nearest(query,1,Agr);
        if(Argb[0].dis<0.0001&&Agr[0].dis<0.0001){
            fprintf(fp_l,"%llf %s %s\n",query[0],depthFiles[i].data(),colorFiles[Argb[0].idx].data());
            fprintf(fp_t,"%llf %lf %lf %lf %lf %lf %lf %lf\n",query[0],grs[7*Agr[0].idx],grs[7*Agr[0].idx+1],
                    grs[7*Agr[0].idx+2],grs[7*Agr[0].idx+3],grs[7*Agr[0].idx+4],grs[7*Agr[0].idx+5],grs[7*Agr[0].idx+6]);
        }
    }
    fclose(fp_l);
    fclose(fp_t);
}


int main(int argc,char **argv)
{
    if(argc!=4){
        printf("error input!\nusage:\ntum2klg tum_unpress_folder trajectory_file_path klg_file_path\n");
        return 0;
    }
    char depth_file_path[400],rgb_file_path[400],groundtruth_file_path[400],list_file_path[400];
    sprintf(depth_file_path,"%s//depth.txt",argv[1]);
    sprintf(rgb_file_path,"%s//rgb.txt",argv[1]);
    sprintf(groundtruth_file_path,"%s//groundtruth.txt",argv[1]);
    sprintf(list_file_path,"%s//list.txt",argv[1]);
    mergeList(depth_file_path,rgb_file_path,groundtruth_file_path,argv[2],list_file_path);
//    mergeList("/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/depth.txt",
//              "/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/rgb.txt",
//              "/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt",
//              "/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/trajectory.txt",
//              "/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/list.txt");
    list data;
    loadList(list_file_path,data);
//    loadList("/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/list.txt",
//          data);
    //FILE *fp=fopen("/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/test.klg","w");
    FILE *fp=fopen(argv[3],"w");
    int total=data.time_stamps.size();int num_pixs1=640*480*2;int num_pixs2=640*480*3;
    fwrite( &total, sizeof(int32_t  ), 1, fp );
    for(int i=0;i<total;++i){
        //printf("%.3f\n",data.time_stamps[i]);
        printf("\r%d/%d",i,total);
        fflush(stdout);
        //cout<<i<<"/"<<total;
        long long int time_stamp=data.time_stamps[i]*100;
        fwrite( &time_stamp, sizeof( int64_t ), 1, fp );
        fwrite( &num_pixs1, sizeof( int32_t ), 1, fp );
        fwrite( &num_pixs2, sizeof( int32_t ), 1, fp );
        char DepthFile[400];
        //cout<<data.Depth_files[i].data()<<endl;
        sprintf(DepthFile,"/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/%s",
                data.Depth_files[i].data());
        //cout<<DepthFile<<endl;
        Mat depth=imread(DepthFile,CV_16UC1);
        //cout<<DepthFile<<endl;
        //cout<<depth.depth()<<endl;
        for(int i=0;i<depth.cols;++i){
            for(int j=0;j<depth.rows;++j){
                depth.at<ushort>(j,i)/=5;
            }
        }
        imshow("depth",depth);
        char *tmp=(char*)depth.data;
        fwrite( tmp, sizeof( char ), num_pixs1, fp );
        sprintf(DepthFile,"/home/yzh/orbslam/SLAM_kinfu/elastic_fusion/rgbd_dataset_freiburg3_long_office_household/%s",
                data.color_files[i].data());

        Mat color=imread(DepthFile);

        imshow("color",color);
        tmp=(char*)color.data;
        fwrite( tmp, sizeof( char ), num_pixs2, fp );
        waitKey(2);
    }
    fclose(fp);
    printf("\n");
    return 0;
}
