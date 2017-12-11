#ifndef LOCALIZATION_FASTMAP_H
#define LOCALIZATION_FASTMAP_H

#if defined( _WIN32 ) || defined( _WIN64 )
#ifdef LIBFASTMAP_EXPORTS
#define LIBFASTMAP_API __declspec(dllexport)
#else
#define LIBFASTMAP_API __declspec(dllimport)
#endif
#else
#define LIBFASTMAP_API
#endif

#include <vector>
#include <string>
#include "basic_struct.h"

extern "C"
{

    LIBFASTMAP_API void fastmap_init();

    LIBFASTMAP_API void init_from_pb(const std::string& pb_path);

    //LIBFASTMAP_API void fastmap_query(double longitude, double latitude, std::string& result);

    LIBFASTMAP_API void fastmap_query(double longitude, double latitude,
                                      std::vector<LANE_COMBO>& result_lanes,
                                      std::vector<ARROW>& result_arrows);

    LIBFASTMAP_API void fastmap_destroy();
}

#endif
