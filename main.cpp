#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>
#include <iostream>
#include <iomanip>


#include "libterraclear/src/filetools.hpp"
#include "libterraclear/src/navmath.h"
#include "libterraclear/src/appsettings.hpp"

namespace tc = terraclear;


//double ft_to_m(double ft)
//{
//    return ft * 0.3048;
//}
//
//double radtodeg(double rad)
//{
//    return (rad * 180.00f) / M_PI;
//}
//
//double degtorad(double deg)
//{
//    return (deg * M_PI) / 180.00f;
//}

//split delimeted string into vector of parts..
std::vector<std::string> split_string(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

// GPS, Status,TWk,Wk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,TimeMS,hAcc
// CAM, GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,Roll,Pitch,Yaw,TimeMS        
enum CAM_PARTS
{
    CAM, 
    GPSTime,
    GPSWeek,
    Unknown1,
    Lat,
    Lng,
    Alt,
    RelAlt,
    Roll,
    Pitch,
    Yaw,
    TimeMS
};

//a map to keep a count of all the entries per type
std::map<std::string, uint32_t> log_line_type_count = {{"CAM",0}, {"GPS",0}};
//vector of all the actual lines that matched the line_types.
std::vector<std::string> log_lines;

void read_log_lines(std::string txtfilepath)
{
    //If log file exist, load them into vector..
    if (tc::filetools::file_exists(txtfilepath.c_str()))
    {
        // log file
        std::ifstream infile(txtfilepath.c_str());
        
        //read all entries from file 
        std::string log_line;
        while (std::getline(infile, log_line))
        {
            //the line type is all the characters up to the first comma..
            std::string line_type_string = log_line.substr(0, log_line.find(","));

            //filter log file line to only those we care about in log_line_type
            //i.e. map key .count will count amount of times line type is found.
            if (log_line_type_count.count(line_type_string) > 0)
            {
                //add logfile line to vector
                log_lines.push_back(log_line);
                
                // increment for each entry.
                log_line_type_count[line_type_string] ++;
            }
        }
    }//endif log file
}

std::vector <std::string> filter_log_lines(std::vector <std::string> log_lines, std::string line_type)
{
    std::vector <std::string> retval;
    for (auto entry_line : log_lines)
    {
        //match extension..
        if ((entry_line.find(line_type) >= 0)
            && (entry_line.substr(0, line_type.length()) == line_type))
        {
                retval.push_back(entry_line);
        }
      
    }

    return retval;
}


int main(int argc, char** argv) 
{
    if (argc < 2)
    {
        std::cout << "ERROR - Path to Flights are missing.. " << std::endl << std::endl;
        std::cout << "Syntax:\tgeofilter [FLIGHT-PATH]" << std::endl << std::endl;
    }

    std::string root_folder = argv[1];

    tc::appsettings cfg("geofilter.json");

    std::vector<float> coords = cfg.getvalue_float_array("coords");
    std::string log_file = tc::filetools::path_append(root_folder, cfg.getvalue_string("log-file"));
    std::string image_src_folder = tc::filetools::path_append(root_folder, cfg.getvalue_string("image-folder"));
    std::string image_keep_folder = tc::filetools::path_append(root_folder, cfg.getvalue_string("image-filter-folder"));

    //get all files from folder
    std::cout << "Reading Images from: " << image_src_folder << std::endl;
    std::vector<std::string> all_files = tc::filetools::read_directory(image_src_folder);
    
    
    //Filter for all supported file types..  i.e. TIFF, JPEG and all variants.. 
    std::vector<std::string> supported_extensions;
    supported_extensions.push_back("jpg");
    supported_extensions.push_back("jpeg");
    supported_extensions.push_back("tif");
    supported_extensions.push_back("tiff");
    supported_extensions.push_back("png");
    supported_extensions.push_back("arw");
    
    //get only filtered files.
    std::vector<std::string> img_files = tc::filetools::filter_files(all_files, supported_extensions, false); 
    
    if (img_files.size() <= 0)
    {
        std::cerr << "\t*ERROR - Image Files not found!" << std::endl << std::endl;
        return -1;
    }
    else
    {
        std::cout << "\tTotal Image Files: " << img_files.size() << std::endl << std::endl;
    }
    
    
    //Read all log file entries, filtered by line type we are looking for.
    std::cout << "Reading Log from: " << log_file << std::endl;
    if (!tc::filetools::file_exists(log_file))
    {
        std::cerr << "\t*ERROR - Log file not found!" << std::endl << std::endl;
        return -1;
    }
    
    //read log files
    read_log_lines(log_file);
    std::vector<std::string> cam_lines = filter_log_lines(log_lines, "CAM");
    std::cout << "\tCAM log entries: " << cam_lines.size() << std::endl << std::endl;

    //make sure we have same amount of log entries as images captured..
    if (img_files.size() != cam_lines.size()) 
    {
        std::cerr << "\n\t*ERROR - Log CAM entries and IMAGE files do NOT match!" << std::endl << std::endl;
        return -1;
    }

    
    bool islat = true;
    float lat_max = -200, lat_min = 200, lon_max = -200, lon_min = 200;
    
    //find GPS corners
    std::vector<tc::navmath::GPSPosition> positions;
    for (float lat_lon : coords)
    {
        if (islat)
        {
            lat_max = (lat_lon > lat_max) ? lat_lon : lat_max;
            lat_min = (lat_lon < lat_min) ? lat_lon : lat_min;
        }
        else
        {
            lon_max = (lat_lon > lon_max) ? lat_lon : lon_max;
            lon_min = (lat_lon < lon_min) ? lat_lon : lon_min;
        }
        
        islat = !islat;
    }
    
    tc::navmath::GPSPosition cnr1;
    cnr1.LatitudeDegrees = lat_max;
    cnr1.LongitudeDegrees = lon_max;
    
    tc::navmath::GPSPosition cnr2;
    cnr2.LatitudeDegrees = lat_min;
    cnr2.LongitudeDegrees = lon_min;
    
    std::cout << "GPS Bounds: " << std::endl;
    std::cout << "\t" << cnr1.LatitudeDegrees << "," << cnr1.LongitudeDegrees << std::endl;
    std::cout << "\t" << cnr2.LatitudeDegrees << "," << cnr2.LongitudeDegrees << std::endl << std::endl;
    
    std::cout << "Moving Filtered Images to: " << image_keep_folder << std::endl << std::endl;

    int img_in = 0;
    int img_out = 0;   
    int match_index = 0;
    for (auto cam_line : cam_lines)
    {
        //match image filename
        std::string image_source = img_files.at(match_index);
        match_index++;

        //trim off all spaces from log entry.
        std::string::iterator end_pos = std::remove(cam_line.begin(), cam_line.end(), ' ');
        cam_line.erase(end_pos, cam_line.end());

        //split string into parts of CAM entry
        // CAM, GPSTime,GPSWeek,Lat,Lng,Alt,RelAlt,Roll,Pitch,Yaw,TimeMS
        std::vector<std::string> cam_parts = split_string(cam_line, ',');
        std::string s_lat = cam_parts.at(CAM_PARTS::Lat);
        std::string s_lon = cam_parts.at(CAM_PARTS::Lng);
//        std::string s_alt_asl = cam_parts.at(CAM_PARTS::Alt);
//        std::string s_alt_rel = cam_parts.at(CAM_PARTS::RelAlt);
//        std::string s_roll = cam_parts.at(CAM_PARTS::Roll);
//        std::string s_pitch = cam_parts.at(CAM_PARTS::Pitch);
//        std::string s_yaw = cam_parts.at(CAM_PARTS::Yaw);     
//        std::string s_gps_time = cam_parts.at(CAM_PARTS::GPSTime);

        float cam_lat = stof(s_lat);
        float cam_lon = stof(s_lon);

        //filter images..
        if ((cam_lat <= lat_max) && (cam_lat >= lat_min) && (cam_lon <= lon_max) && (cam_lon >= lon_min))
        {
            //copy to "IN" folder
            //std::string image_target = tc::filetools::path_append(image_keep_folder, tc::filetools::get_filename(image_source));            
            //tc::filetools::copy_file(image_source, image_target);
            
            //latitude, longitude, name, color
            std::cout << std::fixed << std::setprecision(7) << cam_lat << ", " << std::fixed << std::setprecision(7) << cam_lon << ", rock, FFFF00" << std::endl;
            
            img_in++;
        }
        else
        {
            //latitude, longitude, name, color
            std::cout << std::fixed << std::setprecision(7) << cam_lat << ", " << std::fixed << std::setprecision(7) << cam_lon << ", rock, FFFF00" << std::endl;
            img_out++;
        }
    }
    
    std::cout << std::endl;
    std::cout <<  "Image Retained: " << img_in << std::endl;
    std::cout << "Images Filtered: " << img_out << std::endl << std::endl;
    
    return 0;
}

