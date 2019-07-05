#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <getopt.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/filesystem.hpp"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include "lasfilter.hpp"
#include "simple_ls.hpp"

using namespace std;

/** command line arguments **/

struct CommandLine {

    string file_path;            // -i option, input file
    string folder_input;         // -f option, input folder
    string output_path;          // -o option, output file
    bool shiftZ;
    bool help;                   // -h option, help

} globalArgs;

static const char *optString = "i:o:f:hs";

static const struct option longOpts[] = {
    { "input", required_argument, NULL, 'i' },
    { "folder", required_argument, NULL, 'f' },
    { "output", required_argument, NULL, 'o' },
    { "shift", no_argument, NULL, 's'},
    { "help", no_argument, NULL, 'h' },
    {NULL, no_argument, NULL, 0}
};

void printHelp(){

    cout <<
        "\n"
        "# /*** pcd2laz ***/\n"
        "# /*** Command line arguments ***/\n\n"
        "# -i --input         : input path for single 'pcd' file to be converted\n"
        "# -f --folder        : input folder with all 'pcd' files to be converted\n"
        "# -o --output        : output file path, with explicit format, e.g. file.laz\n"
        "# -s --shift         : shift the Y and Z axes coordinates\n"
        "# -h --help          : print help\n"
    << endl;

        exit(1);
}

string outputFormat(string file){
    std::string current_format = file.substr(file.length()-4, file.length());
    return current_format;
}

double text2number(string number){

    stringstream ss;
    ss << number;

    double num;
    ss >> num;

    return num;
}

double timeStamp(string fileName){
    std::string nameOnly = fileName.substr(0, fileName.length()-4);

    std::size_t found = nameOnly.find_last_of("/\\");
    std::string numStr = nameOnly.substr(found+1);

    return text2number(numStr);
}

int main (int argc, char** argv)
{

    globalArgs.file_path = "";
    globalArgs.folder_input = "";
    globalArgs.output_path = "pcd_merged.laz";
    globalArgs.help = false;
    globalArgs.shiftZ = false;

    int opt = 0;
	int longIndex = 0;

    opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
    while( opt != -1 ) {
        switch( opt ) {
            case 'i':
                globalArgs.file_path = std::string(optarg);
                break;

            case 'o':
                globalArgs.output_path = std::string(optarg);
                break;

            case 'f':
                globalArgs.folder_input = std::string(optarg);
                break;

            case 'h':
                globalArgs.help = true;
                break;

            case 's':
                globalArgs.shiftZ = true;
                break;

            default:
                break;
        }

        opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
    }


    /*** parse command line options ***/

    // print help
    if(globalArgs.help){
        printHelp();
        return 0;
    }

    string format = outputFormat(globalArgs.output_path);
    if(format != ".las" && format != ".laz" && format != ".txt"){
        cout << "## output format not accepted: " << globalArgs.output_path << endl;
        return 0;
    }
    int format_macro;

    if(format == ".las"){
        format_macro = LAS_TOOLS_FORMAT_LAS;
    }else if(format == ".laz"){
        format_macro = LAS_TOOLS_FORMAT_LAZ;
    }else{
        format_macro = LAS_TOOLS_FORMAT_TXT;
    }

    //globalArgs.folder_input = "/home/tiago/Desktop/pcl_temp";
    std::vector<std::string> cloud_files;

    if(globalArgs.file_path != ""){
      string in_format = outputFormat(globalArgs.file_path);
      if(in_format != ".pcd"){
        cout << "## input format not accepted: " << globalArgs.file_path << endl;
        return 0;
      }
      cloud_files.push_back(globalArgs.file_path);
    }else{
      cloud_files = dirFiles(globalArgs.folder_input, ".pcd");
    }

  if(cloud_files.size() == 0){
    cout << "## no 'pcd' files found" << endl;
    exit(1);
  }

  LASwriteOpener laswriteopener;
  laswriteopener.set_file_name(globalArgs.output_path.c_str());
  laswriteopener.set_format(format_macro);

  LASheader lasheader;
  lasheader.point_data_format = 1;
  lasheader.point_data_record_length = 35;
  lasheader.x_scale_factor = 0.0001;
  lasheader.y_scale_factor = 0.0001;
  lasheader.z_scale_factor = 0.0001;

  LASpoint laspoint;
  laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, &lasheader);

  LASwriter* laswriter = laswriteopener.open(&lasheader);

  //cout << "## reading 'pcd' files"<< endl;
  for(unsigned i = 0; i < cloud_files.size(); ++i){

      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

      if (pcl::io::loadPCDFile<pcl::PointXYZI> (cloud_files[i], *cloud) == -1) //* load the file
      {
        //cout << "## could not read file: " << cloud_files[i] << endl;
        PCL_ERROR("");
        return (-1);
      }

      cout << "## reading: " << cloud_files[i] << endl;
      double tstp = timeStamp(cloud_files[i]);
      for (size_t j = 0; j < cloud->points.size (); ++j){

        //alterar parametros!!
        laspoint.set_x(cloud->points[j].x);
        if(globalArgs.shiftZ){
            laspoint.set_y(cloud->points[j].z);
            laspoint.set_z(cloud->points[j].y);
        }else{
            laspoint.set_y(cloud->points[j].y);
            laspoint.set_z(cloud->points[j].z);
        }
        laspoint.set_gps_time(tstp);
        laspoint.set_intensity(cloud->points[j].intensity);

        //cout << cloud->points[j].x << " : " << cloud->points[j].y << " : " << cloud->points[j].z << endl;

        laswriter->write_point(&laspoint);
        laswriter->update_inventory(&laspoint);

      }
  }

  laswriter->update_header(&lasheader, TRUE);
  laswriter->close(TRUE);
  delete laswriter;

  cout << "## output file written at: " << globalArgs.output_path << endl;

  return (0);
}
